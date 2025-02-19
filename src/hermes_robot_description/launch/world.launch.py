import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the package directory
    hermes_robot_description_dir = get_package_share_directory('hermes_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Create the launch description
    ld = LaunchDescription()

    # Launch the simulation with gz sim command
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', 'launch', '-r', PathJoinSubstitution([
            hermes_robot_description_dir,
            'worlds',
            'world2.sdf'
        ])],
        output='screen'
    )

    # Add the command to the launch description
    ld.add_action(gz_sim)

    return ld
