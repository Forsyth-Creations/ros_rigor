from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Declare topic prefixes as a launch argument, expecting a comma-separated list
    prefixes = DeclareLaunchArgument(
        'module_topic_prefixes', 
        default_value="swerve_a,swerve_b,swerve_c,swerve_d", 
        description="Comma-separated list of topic prefixes for the swerve modules"
    )
    
    # Node for the robot, passing the comma-separated topic prefixes as a string
    robot_node = Node(
        package='hermes_robot',  
        executable='robot_node',  
        name='hermes_robot',  
        output='screen',  
        parameters=[{'module_topic_prefixes': LaunchConfiguration('module_topic_prefixes')}],  
    )
    
    # JointStateAggregator
    # joint_state_aggregator = Node(
    #     package='hermes_robot',  
    #     executable='joint_state_aggregator',  
    #     name='joint_state_aggregator',  
    #     output='screen',  
    # )
        
    # Return LaunchDescription with all actions
    return LaunchDescription([
        # joint_state_aggregator,
        prefixes,
        robot_node,
    ])
