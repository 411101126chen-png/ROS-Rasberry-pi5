"""
Launch file for balance car vision nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    vision_pkg_dir = get_package_share_directory('balance_car_vision')
    
    # Configuration file path
    config_file = os.path.join(vision_pkg_dir, 'config', 'vision_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='0',
        description='Camera device index'
    )
    
    # Camera node
    camera_node = Node(
        package='balance_car_vision',
        executable='camera_node',
        name='camera_node',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'camera_device': LaunchConfiguration('camera_device')}
        ],
        output='screen'
    )
    
    # Vision processor node
    vision_processor_node = Node(
        package='balance_car_vision',
        executable='vision_processor',
        name='vision_processor',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        camera_device_arg,
        camera_node,
        vision_processor_node,
    ])
