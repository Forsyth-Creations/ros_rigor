from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Create Four Swerve Modules
    module_a = DeclareLaunchArgument(
        'module_name_a', default_value='swerve_a', description='Namespace for the swerve module'
    )
    
    module_b = DeclareLaunchArgument(
        'module_name_b', default_value='swerve_b', description='Namespace for the swerve module'
    )
    
    module_c = DeclareLaunchArgument(
        'module_name_c', default_value='swerve_c', description='Namespace for the swerve module'
    )
    
    module_d = DeclareLaunchArgument(
        'module_name_d', default_value='swerve_d', description='Namespace for the swerve module'
    )
    
    # Declare topic prefixes as a launch argument, expecting a comma-separated list
    prefixes = DeclareLaunchArgument(
        'module_topic_prefixes', 
        default_value="swerve_a,swerve_b,swerve_c,swerve_d", 
        description="Comma-separated list of topic prefixes for the swerve modules"
    )
    
    # Node for each swerve module
    swerve_module_node_a = Node(
        package='hermes_swerve_module',  
        executable='hermes_swerve_module',  
        name='module_a',  
        output='screen',  
        parameters=[{'module_name': LaunchConfiguration('module_name_a')}],  
    )
    
    swerve_module_node_b = Node(
        package='hermes_swerve_module',  
        executable='hermes_swerve_module',  
        name='module_b',  
        output='screen',  
        parameters=[{'module_name': LaunchConfiguration('module_name_b'), "invert_drive_motor": True}], 
    )
    
    swerve_module_node_c = Node(
        package='hermes_swerve_module',  
        executable='hermes_swerve_module',  
        name='module_c',  
        output='screen',  
        parameters=[{'module_name': LaunchConfiguration('module_name_c')}],  
    )
        
    swerve_module_node_d = Node(
        package='hermes_swerve_module',  
        executable='hermes_swerve_module',  
        name='module_d',  
        output='screen',  
        parameters=[{'module_name': LaunchConfiguration('module_name_d')}],  
    )
    
    # Node for the robot, passing the comma-separated topic prefixes as a string
    robot_node = Node(
        package='hermes_robot',  
        executable='robot_node',  
        name='robot_node',  
        output='screen',  
        parameters=[{'module_topic_prefixes': LaunchConfiguration('module_topic_prefixes')}],  
    )
    
    # JointStateAggregator
    joint_state_aggregator = Node(
        package='hermes_robot',  
        executable='joint_state_aggregator',  
        name='joint_state_aggregator',  
        output='screen',  
    )
    
    pkg_project_description = get_package_share_directory('hermes_robot_description')

    # Include the launch file from the hermes_robot_description package
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_description, 'launch', 'new.launch.py')
        )
    )
    
    # Return LaunchDescription with all actions
    return LaunchDescription([
        robot_description_launch,
        joint_state_aggregator,
        prefixes,
        robot_node,
        module_a,
        module_b,
        module_c,
        module_d,
        swerve_module_node_a,
        swerve_module_node_b,
        swerve_module_node_c,
        swerve_module_node_d
    ])
