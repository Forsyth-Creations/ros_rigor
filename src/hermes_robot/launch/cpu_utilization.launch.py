import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# nav2_vel_cmd_to_swerve.py

def generate_launch_description():
    ld = LaunchDescription()
    
    # Set up the converter node
    
    cpu_utilization = Node(
        package='hermes_robot',
        executable='cpu_utilization',
        name='cpu_utilization',
        output='screen',
    )
    
    ld.add_action(cpu_utilization)
    
    return ld
