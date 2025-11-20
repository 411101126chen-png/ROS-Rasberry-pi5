# Troubleshooting Guide

This guide helps you diagnose and fix common issues with the ROS2 Balance Car system.

## Installation Issues

### ROS2 Not Found
**Symptom:** `ros2: command not found`

**Solution:**
```bash
# Source ROS2 setup file
source /opt/ros/humble/setup.bash

# Add to .bashrc for permanent fix
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Package Not Found After Build
**Symptom:** `Package 'balance_car_control' not found`

**Solution:**
```bash
# Source the workspace
source ~/ros2_balance_car/install/setup.bash

# Add to .bashrc
echo "source ~/ros2_balance_car/install/setup.bash" >> ~/.bashrc
```

### Build Failures
**Symptom:** `colcon build` fails

**Solution:**
```bash
# Install missing dependencies
sudo apt update
sudo apt install python3-colcon-common-extensions

# Install ROS2 dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean and rebuild
rm -rf build install log
colcon build --symlink-install
```

## Camera Issues

### Camera Not Detected
**Symptom:** Camera node fails to start or no image published

**Diagnosis:**
```bash
# Check available video devices
ls -l /dev/video*

# Test camera with v4l2
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

**Solutions:**

1. **Permission issue:**
```bash
sudo usermod -a -G video $USER
# Log out and log back in
```

2. **Wrong device number:**
```bash
# Edit config file
nano src/balance_car_vision/config/vision_params.yaml
# Change camera_device: 0 to camera_device: 1 (or 2, 3...)
```

3. **USB webcam not working:**
```bash
# Test with fswebcam
sudo apt install fswebcam
fswebcam test.jpg
```

4. **Pi Camera not enabled:**
```bash
sudo raspi-config
# Interface Options -> Camera -> Enable
sudo reboot
```

### Low Frame Rate
**Symptom:** Camera publishes < 10 fps

**Solutions:**

1. **Reduce resolution:**
```yaml
# In vision_params.yaml
camera_node:
  ros__parameters:
    image_width: 320   # Instead of 640
    image_height: 240  # Instead of 480
```

2. **USB bandwidth:**
```bash
# Check USB version
lsusb -t
# Use USB 3.0 port if available
```

3. **CPU overload:**
```bash
# Monitor CPU usage
htop
# Disable debug image publishing
vision_processor:
  ros__parameters:
    publish_debug_image: false
```

### Camera Image Frozen
**Symptom:** Image view shows frozen frame

**Diagnosis:**
```bash
# Check if images are being published
ros2 topic hz /camera/image_raw
```

**Solution:**
```bash
# Restart camera node
ros2 node list
ros2 lifecycle set /camera_node shutdown  # If using lifecycle
# Or kill and restart the launch file
```

## Vision Processing Issues

### No Target Detected
**Symptom:** `/vision/target_detected` always False

**Diagnosis:**
```bash
# View debug image
ros2 run rqt_image_view rqt_image_view
# Select /vision/debug_image

# Check target position values
ros2 topic echo /vision/target_position
```

**Solutions:**

1. **Wrong color range (for color detection):**
```yaml
# In vision_params.yaml, try different colors:

# Red:
target_color_lower: [0, 100, 100]
target_color_upper: [10, 255, 255]

# Blue:
target_color_lower: [100, 100, 100]
target_color_upper: [130, 255, 255]

# Green:
target_color_lower: [40, 50, 50]
target_color_upper: [80, 255, 255]

# Yellow:
target_color_lower: [20, 100, 100]
target_color_upper: [30, 255, 255]
```

2. **Poor lighting:**
- Increase ambient light
- Avoid backlighting
- Ensure good contrast between target and background

3. **Target too small:**
```yaml
# Reduce minimum area
vision_processor:
  ros__parameters:
    min_target_area: 100  # Instead of 500
```

4. **Try different detection method:**
```yaml
# Face detection
detection_method: 'face'

# Contour detection
detection_method: 'contour'
```

### Face Detection Not Working
**Symptom:** Face detection method doesn't detect faces

**Solution:**
```bash
# Verify Haar cascade is installed
python3 -c "import cv2; print(cv2.data.haarcascades)"

# Install opencv-contrib if needed
pip3 install opencv-contrib-python

# Ensure good lighting and face is frontal
```

## Control Issues

### Motors Don't Move
**Symptom:** No motor movement despite receiving commands

**Diagnosis:**
```bash
# Check if motor commands are published
ros2 topic echo /left_motor_speed
ros2 topic echo /right_motor_speed

# Check if velocity commands are generated
ros2 topic echo /cmd_vel
```

**Solutions:**

1. **Hardware driver not running:**
```bash
# Check if hardware driver is running
ros2 node list | grep motor

# Start hardware driver
python3 examples/hardware_motor_driver.py
```

2. **No velocity commands:**
```bash
# Check balance controller is running
ros2 node list | grep balance

# Manually test motor controller
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

3. **Hardware issues:**
- Check power supply to motors
- Verify GPIO connections
- Test motors with direct connection
- Check motor driver enable pins

### Motors Spin Wrong Direction
**Symptom:** Robot moves backward when commanded forward

**Solutions:**

1. **Swap motor wires:**
- Physically swap the two wires on one or both motors

2. **Modify motor driver code:**
```python
# In hardware_motor_driver.py, invert direction logic
# Change forward direction pins:
if speed > 0:
    lgpio.gpio_write(self.gpio_handle, dir1_pin, 0)  # Swap
    lgpio.gpio_write(self.gpio_handle, dir2_pin, 1)  # Swap
```

### Balance Car Unstable
**Symptom:** Robot oscillates or tips over

**Solutions:**

1. **Tune PID parameters:**
```yaml
# In control_params.yaml
balance_controller:
  ros__parameters:
    balance_kp: 0.3  # Start lower, increase gradually
    balance_ki: 0.005  # Keep small
    balance_kd: 0.15  # Increase to reduce oscillation
```

2. **Reduce maximum speeds:**
```yaml
max_linear_speed: 0.3  # Instead of 0.5
max_angular_speed: 0.5  # Instead of 1.0
```

3. **Check IMU calibration:**
```bash
# Verify IMU is level and calibrated
ros2 topic echo /imu/data
```

### Target Tracking Not Working
**Symptom:** Robot doesn't follow detected target

**Diagnosis:**
```bash
# Check if tracking is enabled
ros2 param get /balance_controller target_tracking_enabled

# Monitor target position
ros2 topic echo /vision/target_position
```

**Solutions:**

1. **Enable tracking:**
```yaml
# In control_params.yaml
balance_controller:
  ros__parameters:
    target_tracking_enabled: true
```

2. **Target not centered:**
- Move target to center of camera view
- Check if target_position.x is near 0.0

## Performance Issues

### High CPU Usage
**Symptom:** System runs hot or slow

**Solutions:**
```bash
# Monitor processes
htop

# Reduce camera resolution and fps
# Disable debug image publishing
# Lower control loop rates
```

### Delayed Response
**Symptom:** Slow reaction to visual input

**Solutions:**

1. **Reduce processing:**
```yaml
camera_fps: 20.0  # Instead of 30
image_width: 320
image_height: 240
publish_debug_image: false
```

2. **Check topic rates:**
```bash
ros2 topic hz /camera/image_raw
ros2 topic hz /cmd_vel
```

## Network/Communication Issues

### Topics Not Visible
**Symptom:** `ros2 topic list` doesn't show expected topics

**Solutions:**
```bash
# Check if nodes are running
ros2 node list

# Restart DDS daemon
ros2 daemon stop
ros2 daemon start

# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID
# Ensure all terminals use same domain
```

### Node Crashes
**Symptom:** Nodes exit unexpectedly

**Diagnosis:**
```bash
# Run with debug logging
ros2 run balance_car_vision camera_node --ros-args --log-level debug

# Check system logs
journalctl -u ros
```

## Hardware-Specific Issues

### Raspberry Pi 5 GPIO Issues
**Symptom:** lgpio errors or GPIO not responding

**Solutions:**
```bash
# Install/update lgpio
pip3 install --upgrade lgpio

# Check GPIO permissions
sudo usermod -a -G gpio $USER
# Log out and back in

# Test GPIO access
python3 -c "import lgpio; h = lgpio.gpiochip_open(0); print('OK'); lgpio.gpiochip_close(h)"
```

### IMU Not Detected
**Symptom:** I2C device not found

**Solutions:**
```bash
# Enable I2C
sudo raspi-config
# Interface Options -> I2C -> Enable

# Scan I2C bus
sudo apt install i2c-tools
i2cdetect -y 1

# Check I2C speed
# Add to /boot/firmware/config.txt:
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
```

### Power Issues
**Symptom:** Pi resets when motors start

**Solutions:**
- Use separate power supply for motors
- Add large capacitor (1000µF) across motor power
- Ensure common ground between Pi and motor driver
- Check power supply can provide enough current

## Getting More Help

### Enable Debug Logging
```bash
ros2 launch balance_car_full.launch.py --log-level debug
```

### Save Logs
```bash
ros2 launch balance_car_full.launch.py 2>&1 | tee system.log
```

### Record Data for Analysis
```bash
ros2 bag record -a
```

### Check System Resources
```bash
# CPU and memory
htop

# Disk space
df -h

# Temperature (Raspberry Pi)
vcgencmd measure_temp
```

### Community Support
- Open an issue on GitHub
- Check ROS2 documentation: https://docs.ros.org/
- Raspberry Pi forums: https://forums.raspberrypi.com/
- ROS Discourse: https://discourse.ros.org/

## Common Error Messages

### "No module named 'cv2'"
```bash
pip3 install opencv-python
```

### "No module named 'lgpio'"
```bash
pip3 install lgpio
```

### "Failed to open camera device"
```bash
# Check permissions
sudo usermod -a -G video $USER
# Check device exists
ls -l /dev/video*
```

### "Package 'cv_bridge' not found"
```bash
sudo apt install ros-humble-cv-bridge
```

### "Cannot communicate with node"
```bash
# Restart DDS daemon
ros2 daemon stop
ros2 daemon start
```

## Prevention Tips

1. **Always source workspace:**
   - Add to .bashrc: `source ~/ros2_balance_car/install/setup.bash`

2. **Test incrementally:**
   - Test camera alone first
   - Then vision processing
   - Then control system
   - Finally integrate hardware

3. **Use simulation mode:**
   - Test without hardware first
   - Verify software logic before connecting motors

4. **Monitor system health:**
   - Check CPU temperature
   - Monitor CPU usage
   - Verify power supply voltage

5. **Keep backups:**
   - Save working configurations
   - Document parameter changes
   - Use version control (git)
