# Write me a file to launch a world in Gazebo harmonic

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the package directory
    hermes_robot_description_dir = get_package_share_directory('hermes_robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Create the launch description
    ld = LaunchDescription()
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            hermes_robot_description_dir,
            'worlds',
            'world2.sdf'
        ])}.items(),
    )
    
    ld.add_action(gz_sim)

    return ld