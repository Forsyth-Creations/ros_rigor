from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


file_name = "config5_depth.rviz"

def generate_launch_description():
    
    # Get the share directory for the package
    hermes_robot_description_share = FindPackageShare(package='hermes_robot_description').find('hermes_robot_description')
    
    # Declare a launch argument to specify the RViz config file
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([hermes_robot_description_share, 'config', file_name]),
        description='Path to the config file'
    )

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        output='log'
    )

    return LaunchDescription([
        rviz_config_file_arg,
        rviz_node
    ])
