from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Create a new LaunchDescription object
    ld = LaunchDescription()

    # ----------------- Set up the world -----------------
    world_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'world.launch.py'
    )

    # Include the world launch file
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(world_launch_path)
    )
    
    # ----------------- Set up the robot -----------------
    robot_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'robot.launch.py'
    )
    
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_launch_path)
    )
    
    # --------------- Start the bridge node ---------------
    
    bridge_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'bridge.launch.py'
    )
    
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bridge_launch_path)
    )
    
    # ------- Launch RVIZ with the robot model -----------
    rviz_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'rviz.launch.py'
    )
    
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rviz_launch_path)
    )
    
    # ------------ Set up all the modules ------------
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
    )
    
    hermes_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_controller_launch_path)
    )
    
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(nav2_launch_path)
    # )
    
    # Add the action to the LaunchDescription
    ld.add_action(world_launch)
    ld.add_action(robot_launch)
    ld.add_action(rviz_launch)
    ld.add_action(bridge_launch)
    ld.add_action(robot_controller_launch)
    ld.add_action(web_controller_launch)
    ld.add_action(hermes_controller)
    # ld.add_action(nav2_launch)

    return ld
