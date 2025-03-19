from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    nav2_launch_file = os.path.join(nav2_launch_dir, 'bringup_launch.py')
    
    use_slam = "False"
    
    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={'slam': use_slam, 'use_sim_time': "True"}.items(),
        # remappings=[('/cmd_vel', '/nav_cmd_vel')]
    )
    
    ld.add_action(nav2_node)

    return ld
