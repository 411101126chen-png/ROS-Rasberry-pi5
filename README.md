# ROS-Rasberry-pi5

ROS 2 + Raspberry Pi 5 project for visual control and robot navigation.

## Project Structure

```
ROS-Rasberry-pi5/
├── src/
│   ├── follow_pkg/             # Visual following logic (Python)
│   │   └── follow_pkg/
│   │       └── follow.py       # Main node for visual tracking
│   │
│   └── ESP32_connection_pkg/   # Robot description & ESP32 bridge
│       ├── launch/             # Launch files (simulation & real robot)
│       ├── scripts/
│       │   └── esp32_bridge.py # Serial bridge node
│       └── urdf/               # Robot model (Xacro)
│
└── esp32_firmware/             # ESP-IDF Firmware for ESP32
    ├── main/
    │   └── main.c              # Motor control & UART logic
    └── CMakeLists.txt
```

## Changelog

### [2025-11-22] Refactoring & Features
- **Renamed Packages**:
  - `my_first_pkg` -> `follow_pkg`
  - `my_robot_description` -> `ESP32_connection_pkg`
- **Added ESP32 Bridge**:
  - Created `esp32_bridge` node in `ESP32_connection_pkg` to send `cmd_vel` to ESP32 via Serial.
- **Added ESP32 Firmware**:
  - Included `esp32_firmware` (ESP-IDF project) for motor control and UART communication.

## Usage

### 1. Build ROS 2 Workspace
```bash
colcon build
source install/setup.bash
```

### 2. Run Simulation
```bash
ros2 launch ESP32_connection_pkg simulation.launch.py
```

### 3. Run Real Robot (Follow Mode)
```bash
# Terminal 1: Start Bridge
ros2 run ESP32_connection_pkg esp32_bridge.py

# Terminal 2: Start Follow Node
ros2 run follow_pkg follow
```
