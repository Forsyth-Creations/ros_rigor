# This will act as the bridge between the robot and the simulation
# it will utilize gz_bridge to communicate between the robot and the simulation

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    pkg_hermes_robot_description = get_package_share_directory('hermes_robot_description')
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_hermes_robot_description, 'config', 'topic_mapping.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    ld.add_action(bridge)
    
    return ld