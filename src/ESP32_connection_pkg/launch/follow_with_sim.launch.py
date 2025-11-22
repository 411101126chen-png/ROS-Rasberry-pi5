from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_launch = PathJoinSubstitution(
        [FindPackageShare('ESP32_connection_pkg'), 'launch', 'simulation.launch.py']
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='vision_follow.world',
        description='World file (relative to ESP32_connection_pkg/worlds).',
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    follow_node = Node(
        package='follow_pkg',
        executable='follow',
        name='follow_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        simulation,
        follow_node,
    ])
