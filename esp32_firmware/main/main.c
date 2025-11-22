#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ROBOT_CTRL";

// Configuration
#define EX_UART_NUM UART_NUM_0
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

// Motor Control Pins (Example - Change to your actual pins)
// L298N or similar driver
#define GPIO_MOTOR_L_FWD 18
#define GPIO_MOTOR_L_BCK 19
#define GPIO_MOTOR_R_FWD 21
#define GPIO_MOTOR_R_BCK 22

// Global variables for speed
float target_linear_x = 0.0;
float target_angular_z = 0.0;

void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(EX_UART_NUM, &uart_config);
    
    // Set UART pins (using default for UART0, typically TX=1, RX=3 on ESP32)
    // If you use a different UART, change these pins
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void init_motors() {
    gpio_reset_pin(GPIO_MOTOR_L_FWD);
    gpio_reset_pin(GPIO_MOTOR_L_BCK);
    gpio_reset_pin(GPIO_MOTOR_R_FWD);
    gpio_reset_pin(GPIO_MOTOR_R_BCK);
    
    gpio_set_direction(GPIO_MOTOR_L_FWD, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_L_BCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_R_FWD, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_MOTOR_R_BCK, GPIO_MODE_OUTPUT);
}

// Simple differential drive logic
void update_motors() {
    // Calculate left and right wheel speeds
    // V_left = V_linear - (V_angular * Wheel_Base / 2)
    // V_right = V_linear + (V_angular * Wheel_Base / 2)
    // For simplicity, we just add/subtract
    
    float left_speed = target_linear_x - target_angular_z;
    float right_speed = target_linear_x + target_angular_z;

    // Example: Simple ON/OFF based on sign (You should implement PWM here)
    // Left Motor
    if (left_speed > 0.1) {
        gpio_set_level(GPIO_MOTOR_L_FWD, 1);
        gpio_set_level(GPIO_MOTOR_L_BCK, 0);
    } else if (left_speed < -0.1) {
        gpio_set_level(GPIO_MOTOR_L_FWD, 0);
        gpio_set_level(GPIO_MOTOR_L_BCK, 1);
    } else {
        gpio_set_level(GPIO_MOTOR_L_FWD, 0);
        gpio_set_level(GPIO_MOTOR_L_BCK, 0);
    }

    // Right Motor
    if (right_speed > 0.1) {
        gpio_set_level(GPIO_MOTOR_R_FWD, 1);
        gpio_set_level(GPIO_MOTOR_R_BCK, 0);
    } else if (right_speed < -0.1) {
        gpio_set_level(GPIO_MOTOR_R_FWD, 0);
        gpio_set_level(GPIO_MOTOR_R_BCK, 1);
    } else {
        gpio_set_level(GPIO_MOTOR_R_FWD, 0);
        gpio_set_level(GPIO_MOTOR_R_BCK, 0);
    }
}

void rx_task(void *arg) {
    static const char *RX_TAG = "RX_TASK";
    esp_log_level_set(RX_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RD_BUF_SIZE + 1);
    while (1) {
        const int rxBytes = uart_read_bytes(EX_UART_NUM, data, RD_BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            // ESP_LOGI(RX_TAG, "Read %d bytes: '%s'", rxBytes, data);
            
            // Parse command: "L:0.50,A:0.10"
            char *line = strtok((char *)data, "\n");
            while (line != NULL) {
                float l = 0, a = 0;
                if (sscanf(line, "L:%f,A:%f", &l, &a) == 2) {
                    target_linear_x = l;
                    target_angular_z = a;
                    ESP_LOGI(TAG, "Cmd: L=%.2f, A=%.2f", target_linear_x, target_angular_z);
                    update_motors();
                }
                line = strtok(NULL, "\n");
            }
        }
    }
    free(data);
}

void app_main(void) {
    init_uart();
    init_motors();
    xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
}
