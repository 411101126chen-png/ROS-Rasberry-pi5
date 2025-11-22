from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('ESP32_connection_pkg')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='vision_follow.world',
        description='World file (relative to ESP32_connection_pkg/worlds).',
    )

    world_path = PathJoinSubstitution([pkg_share, 'worlds', LaunchConfiguration('world')])
    robot_xacro = PathJoinSubstitution([pkg_share, 'urdf', 'my_robot.urdf.xacro'])
    robot_description = Command(['xacro ', robot_xacro])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        output='screen',
    )

    gz_sim = Node(
        package='ros_gz_sim',
        executable='gzserver',
        name='gzserver',
        arguments=['-r', world_path],
        output='screen',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_my_robot',
        arguments=['-name', 'my_robot', '-topic', 'robot_description', '-x', '0.0', '-y', '0.0', '-z', '0.1'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        rsp_node,
        spawn_entity,
        bridge,
    ])
