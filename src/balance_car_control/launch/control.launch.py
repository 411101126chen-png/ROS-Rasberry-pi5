"""
Launch file for balance car control nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    control_pkg_dir = get_package_share_directory('balance_car_control')
    
    # Configuration file path
    config_file = os.path.join(control_pkg_dir, 'config', 'control_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Motor controller node
    motor_controller_node = Node(
        package='balance_car_control',
        executable='motor_controller',
        name='motor_controller',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Balance controller node
    balance_controller_node = Node(
        package='balance_car_control',
        executable='balance_controller',
        name='balance_controller',
        parameters=[config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        motor_controller_node,
        balance_controller_node,
    ])
