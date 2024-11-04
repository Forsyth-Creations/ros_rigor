import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('hermes_robot')
    world_file = os.path.join(pkg_share, 'worlds', 'empty.world')
    
    
    # this is the launch file for the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_server.launch.py')
    )
    
    # Use an empty world
    empty_world = IncludeLaunchDescription(
    gazebo_rosPackageLaunch,
    launch_arguments={
        'gz_args': '-r -v -v4 empty.sdf',
        'on_exit_shutdown': 'true'
    }.items()
)
    
    # Create the Gazebo node
    gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'gazebo', '-topic', 'ros_description'],
        output='screen'
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add the empty world launch description
    ld.add_action(empty_world)
    
    # Add the Gazebo node
    ld.add_action(gazebo)
    
    return ld
    
    

    
# The launch file is now complete. Save the file and close the editor.

