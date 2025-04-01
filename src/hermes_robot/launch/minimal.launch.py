from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import SetRemap

def generate_launch_description():
    # Create a new LaunchDescription object
    ld = LaunchDescription()

    world_file_name = LaunchConfiguration('world_file_name')

    # ------------------ Add a param for the world file -------------------
    # declare_world_arg = DeclareLaunchArgument(
    #     'world_file_name',
    #     default_value='world2.sdf',
    #     description='Name of the world file'
    # )

    # ----------------- Set up the world -----------------
    # world_launch_path = os.path.join(
    #     get_package_share_directory('hermes_robot_description'), 
    #     'launch', 
    #     'world.launch.py'
    # )

    # # Include the world launch file
    # world_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(world_launch_path),
    #     launch_arguments={'world_file_name': world_file_name}.items()
    # )
    
    # ----------------- Set up the robot -----------------
    robot_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'robot.launch.py'
    ) # This seems to be erroring on large worlds
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path)
    )
    
    # --------------- Start the bridge node ---------------
    
    # bridge_launch_path = os.path.join(
    #     get_package_share_directory('hermes_robot_description'), 
    #     'launch', 
    #     'bridge.launch.py'
    # )
    
    # bridge_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(bridge_launch_path)
    # )
    
    # ------- Launch RVIZ with the robot model -----------
    # rviz_launch_path = os.path.join(
    #     get_package_share_directory('hermes_robot_description'), 
    #     'launch', 
    #     'rviz.launch.py'
    # )
    
    # rviz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(rviz_launch_path)
    # )
    
    # ------------ Set up all the swerve modules ------------
    robot_controller_launch_path = os.path.join(
        get_package_share_directory('hermes_swerve_module'), 
        'launch', 
        'full_robot.launch.py'
    )
    
    robot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_controller_launch_path)
    )
    
    # -------- Launch the Web controller using the launch file ---------
    web_controller_launch_path = os.path.join(
        get_package_share_directory('web_controller'), 
        'launch', 
        'webapp.launch.py'
    )
    
    web_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(web_controller_launch_path)
    )
    
    # ------------ Setup the Robot Controller --------------------
    robot_controller_launch_path = os.path.join(
        get_package_share_directory('hermes_robot'), 
        'launch', 
        'robot.launch.py'
    ) # This is where the odom is coming from
    
    hermes_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_controller_launch_path)
    )
    
    # Launch the Nav2 updater
    
    nav_updater_launch_path = os.path.join(
        get_package_share_directory('hermes_robot'), 
        'launch', 
        'nav_updater.launch.py'
    )
    
    nav_updater_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_updater_launch_path)
    )
    
    # Launch the Realsense Node
    realsense_launch_path = os.path.join(
        get_package_share_directory('realsense2_camera'), 
        'launch', 
        'rs_launch.py'
    )
    
    realsense_launch = GroupAction([
    SetRemap(src='/camera/camera/depth/image_rect_raw', dst='/depth_camera/image_raw'),
    SetRemap(src='/camera/camera/depth/camera_info', dst='/depth_camera/camera_info'),
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={'enable_depth': 'true'}.items()
    )
    ])
    
    
    
    # Add the action to the LaunchDescription
    # ld.add_action(declare_world_arg)
    # ld.add_action(world_launch)
    ld.add_action(robot_launch)
    # ld.add_action(rviz_launch)
    # ld.add_action(bridge_launch)
    ld.add_action(robot_controller_launch)
    ld.add_action(hermes_controller)
    ld.add_action(web_controller_launch)
    ld.add_action(nav_updater_launch)
    # ld.add_action(realsense_launch)

    return ld
