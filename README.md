# ROS 2 SBR Project

This project contains ROS 2 nodes for a Self-Balancing Robot (SBR) and the corresponding ESP32 firmware.

## Project Structure

- `sbr_nodes/`: ROS 2 Python package.
  - `vision_commander`: Reads from `/dev/video0` and publishes `/cmd_vel`.
  - `serial_bridge`: Subscribes to `/cmd_vel` and sends commands to ESP32 via Serial.
- `esp32_firmware/`: PlatformIO project for ESP32 (Arduino framework).
  - `src/main.cpp`: Firmware code to receive serial commands.

## Usage

### 1. ROS 2 Nodes

1.  **Build the package:**
    ```bash
    cd ~/Ros2程式碼/Ros2-SBR-esp32
    colcon build --packages-select sbr_nodes
    source install/setup.bash
    ```

2.  **Run Vision Commander:**
    ```bash
    ros2 run sbr_nodes vision_commander
    ```

3.  **Run Serial Bridge:**
    ```bash
    # Default port is /dev/ttyUSB0
    ros2 run sbr_nodes serial_bridge --ros-args -p port:=/dev/ttyUSB0
    ```

### 2. ESP32 Firmware

1.  **Open `esp32_firmware` in PlatformIO (VS Code).**
2.  **Build and Upload** to your ESP32.
3.  **Monitor Serial** to verify connection (Baudrate: 115200).

## Notes

- **Vision Logic**: The `vision_commander.py` file contains a placeholder for your custom vision logic. Look for the `[USER TODO]` comment.
- **Safety**: The ESP32 firmware includes a 1-second timeout. If no command is received, `target_velocity` is set to 0.
