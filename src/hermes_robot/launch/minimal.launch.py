from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Launch Argument for Simulation Mode
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode', 
        default_value='all', 
        description='"robot" to run robot nodes, "world" for world nodes, "all" for both'
    )
    
    # Launch Argument for World File
    world_file_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='world2.sdf',
        description='Name of the world file'
    )
    world_file_name = LaunchConfiguration('world_file_name')
    
    # Function to Include Launch Files
    def include_launch(package, launch_file, launch_args=None):
        launch_path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments=launch_args.items() if launch_args else None
        )
    
    # Robot Launch Files
    robot_description = include_launch('hermes_robot_description', 'robot_description_w_gazebo.launch.py')
    robot_additional_nodes = include_launch('hermes_robot_description', 'robot_additional_nodes.launch.py')
    robot_controller = include_launch('hermes_swerve_module', 'full_robot.launch.py')
    hermes_controller = include_launch('hermes_robot', 'robot.launch.py')
    nav_updater = include_launch('hermes_robot', 'nav_updater.launch.py')
    
    # World Setup
    world_launch = include_launch('hermes_robot_description', 'world.launch.py', {'world_file_name': world_file_name})
    
    # Visualization & Bridge
    rviz_launch = include_launch('hermes_robot_description', 'rviz.launch.py')
    bridge_launch = include_launch('hermes_robot_description', 'bridge.launch.py')
    
    # Web Controller
    web_controller = include_launch('web_controller', 'webapp.launch.py')
    
    # Realsense Node with Remapping
    realsense_launch = GroupAction([
        SetRemap(src='/camera/camera/depth/image_rect_raw', dst='/depth_camera/image_raw'),
        SetRemap(src='/camera/camera/depth/camera_info', dst='/depth_camera/camera_info'),
        include_launch('realsense2_camera', 'rs_launch.py', {'enable_depth': 'true'})
    ])
    
    # Add Launch Arguments
    ld.add_action(simulation_mode_arg)
    ld.add_action(world_file_arg)
    
    # Retrieve Simulation Mode
    simulation_mode = LaunchConfiguration('simulation_mode')
    
    # Add Actions Based on Simulation Mode
    if simulation_mode in ['robot', 'all']:
        ld.add_action(robot_additional_nodes)
        ld.add_action(robot_controller)
        ld.add_action(hermes_controller)
        ld.add_action(web_controller)
        ld.add_action(nav_updater)
    
    if simulation_mode in ['world', 'all']:
        ld.add_action(robot_description)
        ld.add_action(world_launch)
        ld.add_action(rviz_launch)
        ld.add_action(bridge_launch)
        ld.add_action(realsense_launch)
    
    return ld
