# Quick Start Guide

## 1. First Time Setup (5-10 minutes)

### Prerequisites Check
```bash
# Check if ROS2 is installed
ros2 --version

# Check if Python 3 is installed
python3 --version

# Check camera is detected
ls /dev/video*
```

### Installation
```bash
# Clone the repository
cd ~
git clone https://github.com/411101126chen-png/ROS-Rasberry-pi5.git ros2_balance_car
cd ros2_balance_car

# Run setup script
source /opt/ros/humble/setup.bash  # Or your ROS2 distro
./setup.sh
```

## 2. Test Camera (2 minutes)

```bash
# Source workspace
source ~/ros2_balance_car/install/setup.bash

# Launch only the camera node
ros2 run balance_car_vision camera_node

# In another terminal, view the camera feed
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw
```

**✓ You should see your webcam video feed**

## 3. Test Vision Processing (3 minutes)

```bash
# Launch vision system
ros2 launch balance_car_vision vision.launch.py

# In another terminal, view debug image
ros2 run rqt_image_view rqt_image_view
# Select topic: /vision/debug_image
```

**✓ Hold a red object in front of camera - you should see it highlighted**

### Adjust Color Detection
If red detection doesn't work well, edit the config file:
```bash
nano src/balance_car_vision/config/vision_params.yaml
```

Try these colors:
- **Blue ball**: `target_color_lower: [100, 100, 100]`, `target_color_upper: [130, 255, 255]`
- **Green object**: `target_color_lower: [40, 50, 50]`, `target_color_upper: [80, 255, 255]`

## 4. Monitor Target Detection (2 minutes)

```bash
# Check if target is being detected
ros2 topic echo /vision/target_detected

# Check target position
ros2 topic echo /vision/target_position
```

**✓ Move colored object left/right - x value should change from -1 to 1**

## 5. Test Full System (5 minutes)

```bash
# Launch everything
source ~/ros2_balance_car/install/setup.bash
ros2 launch balance_car_full.launch.py
```

**Monitor the system:**
```bash
# In separate terminals:

# 1. View all active nodes
ros2 node list

# 2. Monitor velocity commands
ros2 topic echo /cmd_vel

# 3. Check motor speeds
ros2 topic echo /left_motor_speed
ros2 topic echo /right_motor_speed
```

## 6. Connect Hardware Motors

The system is now ready! To connect real motors:

1. **Connect your motor driver** to Raspberry Pi GPIO pins
2. **Create a motor driver node** that subscribes to:
   - `/left_motor_speed` (Float32)
   - `/right_motor_speed` (Float32)
3. **Control GPIO** to drive motors based on these speed values

Example using lgpio (Raspberry Pi 5):
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio

class HardwareMotorDriver(Node):
    def __init__(self):
        super().__init__('hardware_motor_driver')
        
        # Open GPIO chip
        self.h = lgpio.gpiochip_open(0)
        
        # Setup motor pins (adjust to your wiring)
        self.LEFT_PWM = 12
        self.LEFT_DIR = 16
        self.RIGHT_PWM = 13
        self.RIGHT_DIR = 18
        
        # ... setup PWM and direction pins ...
        
        self.create_subscription(Float32, 'left_motor_speed', self.left_callback, 10)
        self.create_subscription(Float32, 'right_motor_speed', self.right_callback, 10)
    
    def left_callback(self, msg):
        speed = msg.data
        # Convert speed to PWM duty cycle and set direction
        # ... your motor control code ...
```

## Common Issues

### Camera not detected
```bash
# Check camera permissions
sudo usermod -a -G video $USER
# Log out and log back in
```

### No target detected
- Check lighting - need good contrast
- Adjust HSV color ranges in config
- Try 'face' detection method instead of 'color'

### Low frame rate
- Reduce camera resolution in config: `image_width: 320`, `image_height: 240`
- Reduce fps: `camera_fps: 15.0`

## Next Steps

1. **Tune PID parameters** in `src/balance_car_control/config/control_params.yaml`
2. **Add IMU sensor** for better balance control
3. **Implement motor hardware driver** for your specific motor controller
4. **Experiment with different detection methods** (color, face, contour)
5. **Create custom detection algorithms** for your specific use case

## Getting Help

- Check the main [README.md](README.md) for detailed documentation
- Review ROS2 logs: `ros2 launch balance_car_full.launch.py --log-level debug`
- Open an issue on GitHub if you encounter problems
