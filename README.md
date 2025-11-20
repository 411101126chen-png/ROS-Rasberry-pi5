# ROS2 Balance Car with Webcam Visual Control for Raspberry Pi 5

This project implements a ROS2-based balance car system with webcam visual feedback running on Raspberry Pi 5. The system uses visual processing to detect and track targets, allowing the balance car to follow objects or maintain balance based on visual input.

## Features

- 🚗 **Balance Car Control**: PID-based balance controller with motor control
- 📷 **Webcam Integration**: Real-time video capture and processing
- 👁️ **Visual Target Tracking**: Multiple detection methods (color, contour, face)
- 🎯 **Autonomous Navigation**: Follow detected targets automatically
- ⚙️ **Configurable Parameters**: Easy-to-adjust control and vision parameters
- 🔧 **ROS2 Architecture**: Modular design with separate nodes for each function

## System Architecture

```
┌─────────────────┐
│  Webcam Camera  │
└────────┬────────┘
         │ Image Feed
         ▼
┌─────────────────┐      ┌──────────────────┐
│  Camera Node    │─────▶│ Vision Processor │
└─────────────────┘      └────────┬─────────┘
                                  │ Target Position
                                  ▼
                         ┌─────────────────────┐
                         │ Balance Controller  │◀──── IMU Data
                         └──────────┬──────────┘
                                    │ Velocity Commands
                                    ▼
                         ┌─────────────────────┐
                         │  Motor Controller   │
                         └──────────┬──────────┘
                                    │ Motor Speeds
                                    ▼
                         ┌─────────────────────┐
                         │  Balance Car Motors │
                         └─────────────────────┘
```

## Requirements

### Hardware
- Raspberry Pi 5 (4GB or 8GB recommended)
- USB Webcam or Raspberry Pi Camera Module
- Balance car chassis with DC motors
- Motor driver (e.g., L298N, TB6612FNG)
- IMU sensor (e.g., MPU6050, BNO055) - optional but recommended
- Power supply for motors and Pi

### Software
- Ubuntu 22.04 or later (64-bit)
- ROS2 Humble or newer
- Python 3.8+
- OpenCV (cv2)
- NumPy

## Installation

### 1. Install ROS2 on Raspberry Pi 5

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop -y

# Install additional dependencies
sudo apt install python3-colcon-common-extensions python3-pip -y
sudo apt install ros-humble-cv-bridge ros-humble-image-transport -y
```

### 2. Install Python Dependencies

```bash
pip3 install opencv-python numpy
```

### 3. Clone and Build This Repository

```bash
# Create workspace and clone
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone https://github.com/411101126chen-png/ROS-Rasberry-pi5.git .

# Source ROS2
source /opt/ros/humble/setup.bash

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Configuration

### Camera Configuration

Edit `src/balance_car_vision/config/vision_params.yaml` to configure camera settings:

```yaml
camera_node:
  ros__parameters:
    camera_device: 0        # Change to appropriate camera device
    camera_fps: 30.0
    image_width: 640
    image_height: 480
```

### Vision Detection Configuration

Choose detection method and configure parameters:

```yaml
vision_processor:
  ros__parameters:
    detection_method: 'color'  # Options: 'color', 'contour', 'face'
    target_color_lower: [0, 100, 100]    # Red color (HSV)
    target_color_upper: [10, 255, 255]
```

**Color Detection Examples:**
- **Red**: lower: [0, 100, 100], upper: [10, 255, 255]
- **Blue**: lower: [100, 100, 100], upper: [130, 255, 255]
- **Green**: lower: [40, 50, 50], upper: [80, 255, 255]
- **Yellow**: lower: [20, 100, 100], upper: [30, 255, 255]

### Control Parameters

Edit `src/balance_car_control/config/control_params.yaml`:

```yaml
balance_controller:
  ros__parameters:
    balance_kp: 0.5           # Proportional gain
    balance_ki: 0.01          # Integral gain
    balance_kd: 0.1           # Derivative gain
    max_linear_speed: 0.5     # m/s
    max_angular_speed: 1.0    # rad/s
```

## Usage

### Launch the Complete System

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch all nodes
ros2 launch balance_car_full.launch.py
```

### Launch Individual Components

**Vision System Only:**
```bash
ros2 launch balance_car_vision vision.launch.py
```

**Control System Only:**
```bash
ros2 launch balance_car_control control.launch.py
```

**Individual Nodes:**
```bash
# Camera node
ros2 run balance_car_vision camera_node

# Vision processor
ros2 run balance_car_vision vision_processor

# Balance controller
ros2 run balance_car_control balance_controller

# Motor controller
ros2 run balance_car_control motor_controller
```

### Monitoring and Debugging

**View Camera Feed:**
```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw or /vision/debug_image
```

**Check Topics:**
```bash
ros2 topic list
ros2 topic echo /vision/target_position
ros2 topic echo /vision/target_detected
ros2 topic echo /cmd_vel
```

**Monitor Node Status:**
```bash
ros2 node list
ros2 node info /vision_processor
```

**Visualize with RViz2:**
```bash
ros2 run rviz2 rviz2
```

## ROS2 Topics

### Published Topics
- `/camera/image_raw` (sensor_msgs/Image): Raw camera feed
- `/vision/debug_image` (sensor_msgs/Image): Annotated image with detected targets
- `/vision/target_position` (geometry_msgs/Point): Normalized target position (-1 to 1)
- `/vision/target_detected` (std_msgs/Bool): Target detection status
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for motors
- `/left_motor_speed` (std_msgs/Float32): Left motor speed
- `/right_motor_speed` (std_msgs/Float32): Right motor speed

### Subscribed Topics
- `/imu/data` (sensor_msgs/Imu): IMU data for balance control (optional)

## Customization

### Adding New Detection Methods

Edit `src/balance_car_vision/balance_car_vision/vision_processor.py` and add your custom detection function:

```python
def detect_custom_target(self, image):
    # Your detection logic here
    target_center = None
    debug_image = image.copy()
    
    # ... detection code ...
    
    return target_center, debug_image
```

### Motor Driver Integration

The `motor_controller` node publishes motor speeds. You'll need to add hardware-specific code to interface with your motor driver. This can be done by:

1. Creating a motor driver node that subscribes to `/left_motor_speed` and `/right_motor_speed`
2. Using GPIO libraries (e.g., RPi.GPIO, lgpio) to control motor pins
3. Implementing PWM control for motor speed

Example integration:
```python
import lgpio  # or RPi.GPIO

# Subscribe to motor speed topics
# Convert speed values to PWM signals
# Control GPIO pins accordingly
```

## Troubleshooting

### Camera Not Detected
```bash
# List available cameras
ls -l /dev/video*

# Test camera with v4l2
v4l2-ctl --list-devices

# Update camera_device parameter in config
```

### No Target Detected
- Ensure proper lighting conditions
- Adjust HSV color ranges in configuration
- Check camera focus and positioning
- View debug image to verify detection algorithm

### Balance Issues
- Tune PID parameters (Kp, Ki, Kd)
- Verify IMU calibration and orientation
- Check motor connections and power supply
- Adjust speed limits

### Permission Issues with Camera
```bash
sudo usermod -a -G video $USER
# Logout and login again
```

## Development

### Adding Features
1. Create new nodes in appropriate packages
2. Update `setup.py` entry points
3. Create/update launch files
4. Document new parameters in config files

### Testing Individual Nodes
```bash
# Build specific package
colcon build --packages-select balance_car_vision

# Test with ros2 run
ros2 run balance_car_vision vision_processor
```

## Contributing

Contributions are welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

MIT License - See LICENSE file for details

## Acknowledgments

- ROS2 Community
- OpenCV Contributors
- Raspberry Pi Foundation

## Support

For issues and questions:
- Open an issue on GitHub
- Check ROS2 documentation: https://docs.ros.org/
- Raspberry Pi forums: https://forums.raspberrypi.com/

## Future Enhancements

- [ ] Add deep learning-based object detection (YOLO, MobileNet)
- [ ] Implement SLAM for mapping and navigation
- [ ] Add obstacle avoidance
- [ ] Web-based control interface
- [ ] Multi-camera support
- [ ] Automated tuning of PID parameters
- [ ] Data logging and analysis tools
