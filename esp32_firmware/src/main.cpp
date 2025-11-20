#include <Arduino.h>

// Global variables for control
float target_velocity = 0.0;
float target_steering = 0.0;

// Safety timeout variables
unsigned long last_command_time = 0;
const unsigned long COMMAND_TIMEOUT_MS = 1000;

void receive_command() {
    // Check if data is available on Serial
    if (Serial.available() > 0) {
        // Read the string until newline
        String inputString = Serial.readStringUntil('\n');
        inputString.trim(); // Remove any leading/trailing whitespace

        // Expected format: "linear_x,angular_z" (e.g., "0.25,-0.10")
        int commaIndex = inputString.indexOf(',');
        
        if (commaIndex > 0) {
            String val1 = inputString.substring(0, commaIndex);
            String val2 = inputString.substring(commaIndex + 1);

            // Parse floats
            float new_velocity = val1.toFloat();
            float new_steering = val2.toFloat();

            // Update global variables
            target_velocity = new_velocity;
            target_steering = new_steering;

            // Update timestamp for safety timeout
            last_command_time = millis();
            
            // Debug output (optional, comment out in production if it interferes)
            // Serial.print("Updated: v=");
            // Serial.print(target_velocity);
            // Serial.print(", s=");
            // Serial.println(target_steering);
        }
    }
}

void setup() {
    // Initialize Serial
    Serial.begin(115200);
    
    // Wait for serial port to connect (needed for native USB port only)
    // while (!Serial) {
    //   ; 
    // }
    
    // Initialize other hardware (motors, IMU, etc.) here
}

void loop() {
    // 1. Receive and parse commands
    receive_command();

    // 2. Safety Timeout Check
    if (millis() - last_command_time > COMMAND_TIMEOUT_MS) {
        target_velocity = 0.0;
        target_steering = 0.0;
        // Optional: You might want to keep steering active or set it to 0 too.
        // Prompt asked to "auto zero target_velocity".
        // I also zeroed steering for safety, but you can adjust if needed.
    }

    // 3. PID Control Loop (Your existing logic would go here)
    // ... use target_velocity and target_steering ...
    
    // Example delay to prevent loop from running too fast (adjust as needed)
    delay(10); 
}
