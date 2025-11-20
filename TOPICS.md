# ROS2 System Architecture and Topics

## Node Graph

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        ROS2 Balance Car System                           │
└─────────────────────────────────────────────────────────────────────────┘

┌────────────────┐
│  camera_node   │
│ (vision pkg)   │
└────────┬───────┘
         │
         │ /camera/image_raw
         │ (sensor_msgs/Image)
         │
         ▼
┌──────────────────────┐
│  vision_processor    │
│  (vision pkg)        │
└───────┬──────────────┘
        │
        ├─ /vision/target_position (geometry_msgs/Point)
        ├─ /vision/target_detected (std_msgs/Bool)
        └─ /vision/debug_image (sensor_msgs/Image) [optional]
        │
        │
        ▼
┌──────────────────────┐       /imu/data
│ balance_controller   │◄─────── (sensor_msgs/Imu) [optional]
│  (control pkg)       │
└───────┬──────────────┘
        │
        │ /cmd_vel
        │ (geometry_msgs/Twist)
        │
        ▼
┌──────────────────────┐
│  motor_controller    │
│  (control pkg)       │
└───────┬──────────────┘
        │
        ├─ /left_motor_speed (std_msgs/Float32)
        └─ /right_motor_speed (std_msgs/Float32)
        │
        │
        ▼
┌──────────────────────┐
│ hardware_motor_driver│
│   (your hardware)    │
└──────────────────────┘
        │
        ▼
   [Physical Motors]
```

## Topic Details

### Published Topics

| Topic | Message Type | Publisher | Description | Rate |
|-------|-------------|-----------|-------------|------|
| `/camera/image_raw` | sensor_msgs/Image | camera_node | Raw camera frames | 30 Hz |
| `/vision/debug_image` | sensor_msgs/Image | vision_processor | Annotated debug visualization | 30 Hz |
| `/vision/target_position` | geometry_msgs/Point | vision_processor | Target position (normalized -1 to 1) | ~30 Hz |
| `/vision/target_detected` | std_msgs/Bool | vision_processor | Detection status | ~30 Hz |
| `/cmd_vel` | geometry_msgs/Twist | balance_controller | Velocity commands | 50 Hz |
| `/left_motor_speed` | std_msgs/Float32 | motor_controller | Left motor speed (-1 to 1) | 50 Hz |
| `/right_motor_speed` | std_msgs/Float32 | motor_controller | Right motor speed (-1 to 1) | 50 Hz |

### Subscribed Topics

| Topic | Message Type | Subscriber | Description |
|-------|-------------|-----------|-------------|
| `/camera/image_raw` | sensor_msgs/Image | vision_processor | Input for visual processing |
| `/vision/target_position` | geometry_msgs/Point | balance_controller | Target tracking input |
| `/vision/target_detected` | std_msgs/Bool | balance_controller | Detection status input |
| `/imu/data` | sensor_msgs/Imu | balance_controller | IMU data for balance (optional) |
| `/cmd_vel` | geometry_msgs/Twist | motor_controller | Velocity commands to execute |
| `/left_motor_speed` | std_msgs/Float32 | hardware_motor_driver | Left motor control |
| `/right_motor_speed` | std_msgs/Float32 | hardware_motor_driver | Right motor control |

## Message Format Details

### geometry_msgs/Point (target_position)
```yaml
x: float  # Horizontal position: -1 (left) to 1 (right), 0 = center
y: float  # Vertical position: -1 (top) to 1 (bottom), 0 = center
z: float  # Not used (set to 0.0)
```

### geometry_msgs/Twist (cmd_vel)
```yaml
linear:
  x: float  # Forward/backward velocity (m/s)
  y: float  # Not used (set to 0.0)
  z: float  # Not used (set to 0.0)
angular:
  x: float  # Not used (set to 0.0)
  y: float  # Not used (set to 0.0)
  z: float  # Rotational velocity (rad/s)
```

### std_msgs/Float32 (motor speeds)
```yaml
data: float  # Motor speed: -1.0 (full reverse) to 1.0 (full forward)
```

## Data Flow

1. **Image Acquisition**: Camera captures frames and publishes to `/camera/image_raw`
2. **Visual Processing**: Vision processor detects targets and publishes position/status
3. **Balance Control**: Balance controller combines vision data and IMU (if available) to compute motion
4. **Motor Control**: Motor controller converts Twist commands to individual motor speeds
5. **Hardware Interface**: Hardware driver translates motor speeds to GPIO PWM signals
6. **Physical Motion**: Motors drive the balance car

## Monitoring Topics

### View all topics
```bash
ros2 topic list
```

### Monitor specific topic
```bash
# Target detection status
ros2 topic echo /vision/target_detected

# Target position
ros2 topic echo /vision/target_position

# Velocity commands
ros2 topic echo /cmd_vel

# Motor speeds
ros2 topic echo /left_motor_speed
ros2 topic echo /right_motor_speed
```

### Topic statistics
```bash
# Show publication rate
ros2 topic hz /camera/image_raw

# Show topic info
ros2 topic info /cmd_vel

# Show message details
ros2 interface show geometry_msgs/msg/Twist
```

## Visualization

### Using rqt_graph
```bash
ros2 run rqt_graph rqt_graph
```
This shows the node/topic graph visually.

### Using rqt_image_view
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/camera/image_raw` or `/vision/debug_image` to view camera feed.

### Using rqt_plot
```bash
ros2 run rqt_plot rqt_plot
```
Plot motor speeds or target position over time.

## Custom Topic Integration

### Adding IMU Support

To add IMU data:

1. Install IMU ROS2 driver (e.g., `ros-humble-imu-tools`)
2. Configure IMU to publish to `/imu/data`
3. The balance controller already subscribes to this topic

Example IMU launch:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_tools',
            executable='imu_node',
            name='imu_node',
            parameters=[{'port': '/dev/ttyUSB0'}]
        )
    ])
```

### Publishing Manual Commands

For testing, publish manual commands:
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Debugging

### Check if nodes are running
```bash
ros2 node list
```

### Check node details
```bash
ros2 node info /camera_node
ros2 node info /vision_processor
ros2 node info /balance_controller
ros2 node info /motor_controller
```

### Monitor all topics simultaneously
```bash
ros2 topic echo /vision/target_detected &
ros2 topic echo /cmd_vel &
ros2 topic hz /camera/image_raw
```

### Record and playback data
```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /camera/image_raw /vision/target_position /cmd_vel

# Playback
ros2 bag play <bag_file>
```
