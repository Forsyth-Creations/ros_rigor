import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get the package directory
    hermes_robot_description_dir = get_package_share_directory('hermes_robot_description')

    world_file_name = LaunchConfiguration('world_file_name')
    
    # Create the launch description
    ld = LaunchDescription()

    # Declare a world argument
    declare_world_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value="world2.sdf",
        description='Path to the world file'
    )
    
    # Print out the location of the world file
    print(f"World file location: {world_file_name}")

    # Launch the simulation with gz sim command
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', 'launch', '-r', PathJoinSubstitution([
            hermes_robot_description_dir,
            'worlds',
            world_file_name
        ])],
        output='screen'
    )

    # Add the command to the launch description
    ld.add_action(declare_world_arg)
    ld.add_action(gz_sim)

    return ld
