"""
Main launch file for balance car with webcam visual simulation
Launches both control and vision systems
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Include control launch file
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('balance_car_control'),
            '/launch/control.launch.py'
        ])
    )
    
    # Include vision launch file
    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('balance_car_vision'),
            '/launch/vision.launch.py'
        ])
    )
    
    return LaunchDescription([
        control_launch,
        vision_launch,
    ])
