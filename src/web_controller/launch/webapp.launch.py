import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Get the lib directory for the package
    package_prefix = get_package_prefix('web_controller')
    lib_directory = os.path.join(package_prefix, 'lib', 'web_controller', 'backend', 'server.py')

    # Ensure the path is correctly constructed
    print(f"FastAPI server path: {lib_directory}")
    
    ld = LaunchDescription()


    # Declare any launch arguments if needed (e.g., for FastAPI port, etc.)

    # Start the ROS 2 node (RobotPublisher)
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'run', 'web_controller', 'robot_publisher_node'],
        name='robot_publisher_node',
        output='screen',
        shell=True,
        sigkill_timeout = "1"
    ))

    # Start the FastAPI backend as a separate process
    ld.add_action(ExecuteProcess(
        cmd=['python3', lib_directory],
        name='fastapi_server',
        output='screen',
        shell=True,
        sigkill_timeout = "0.5"
    ))
    
    # Change to the frontend directory and start the frontend
    ld.add_action(ExecuteProcess(
        cmd=['npm', 'run', 'start'],
        name='frontend',
        output='screen',
        shell=True,
        cwd=[os.path.join(package_prefix, 'share', 'web_controller', 'frontend')],
        sigkill_timeout = "0.5"
    ))
    
    
    
    return ld

    
