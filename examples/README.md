# Examples

This directory contains example implementations and helper scripts for the ROS2 Balance Car system.

## Hardware Motor Driver Example

### `hardware_motor_driver.py`

A complete example implementation of a hardware motor driver for Raspberry Pi 5 using the `lgpio` library. This driver:

- Subscribes to `/left_motor_speed` and `/right_motor_speed` topics
- Controls GPIO pins to drive motors via L298N motor driver
- Supports both forward and reverse directions
- Uses hardware PWM for smooth speed control
- Includes simulation mode when GPIO is not available

#### Prerequisites

```bash
# Install lgpio for Raspberry Pi 5
pip3 install lgpio
```

#### Usage

**Standalone mode:**
```bash
python3 hardware_motor_driver.py
```

**With ROS2:**
```bash
# In one terminal, start the motor driver
python3 hardware_motor_driver.py

# In another terminal, test by publishing motor commands
ros2 topic pub /left_motor_speed std_msgs/msg/Float32 "data: 0.5"
ros2 topic pub /right_motor_speed std_msgs/msg/Float32 "data: 0.5"
```

**Integrated with the balance car system:**
```bash
# Terminal 1: Run the main system
ros2 launch balance_car_full.launch.py

# Terminal 2: Run the hardware driver
python3 examples/hardware_motor_driver.py
```

#### Pin Configuration

Default GPIO pin assignments (can be modified via ROS2 parameters):

| Function | GPIO Pin | Physical Pin |
|----------|----------|--------------|
| Left Motor PWM | GPIO 12 | Pin 32 |
| Left Motor DIR1 | GPIO 16 | Pin 36 |
| Left Motor DIR2 | GPIO 20 | Pin 38 |
| Right Motor PWM | GPIO 13 | Pin 33 |
| Right Motor DIR1 | GPIO 19 | Pin 35 |
| Right Motor DIR2 | GPIO 26 | Pin 37 |

#### Customization

To change pin assignments, modify the parameters in the code or pass them as ROS2 parameters:

```python
# In your launch file or command line
parameters=[{
    'left_motor_pwm_pin': 12,
    'left_motor_dir1_pin': 16,
    'left_motor_dir2_pin': 20,
    'right_motor_pwm_pin': 13,
    'right_motor_dir1_pin': 19,
    'right_motor_dir2_pin': 26,
    'pwm_frequency': 1000
}]
```

#### Adapting for Other Motor Drivers

This example is configured for L298N. To adapt for other drivers:

**TB6612FNG:**
- Similar interface to L298N
- May need to adjust PWM frequency
- Check datasheet for optimal frequency

**DRV8833:**
- Uses two pins per motor (no separate enable)
- Modify direction control logic
- PWM on one pin, direction on the other

**Other drivers:**
- Refer to your motor driver datasheet
- Adjust pin configuration and control logic accordingly

## Adding Your Own Examples

To contribute additional examples:

1. Create a new Python file in this directory
2. Include comprehensive docstrings
3. Make it executable: `chmod +x your_example.py`
4. Document it in this README
5. Test thoroughly before submitting

## Future Examples (Contributions Welcome)

- IMU sensor integration (MPU6050, BNO055)
- Wheel encoder reading for odometry
- Battery voltage monitoring
- Emergency stop implementation
- Remote control via joystick/gamepad
- Automated calibration scripts
- Data logging and visualization
- Deep learning model integration
