import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# nav2_vel_cmd_to_swerve.py

def generate_launch_description():
    ld = LaunchDescription()
    
    # Set up the converter node
    
    temperature_node = Node(
        package='hermes_robot',
        executable='temperature_node',
        name='temperature_node',
        output='screen',
    )
    
    ld.add_action(temperature_node)
    
    return ld
