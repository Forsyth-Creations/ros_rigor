import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    # Get the lib directory for the package
    package_prefix = get_package_prefix('web_controller')
    lib_directory = os.path.join(package_prefix, 'lib', 'web_controller', 'server.py')

    # Ensure the path is correctly constructed
    print(f"FastAPI server path: {lib_directory}")

    return LaunchDescription([
        # Declare any launch arguments if needed (e.g., for FastAPI port, etc.)

        # Start the ROS 2 node (RobotPublisher)
        ExecuteProcess(
            cmd=['ros2', 'run', 'web_controller', 'robot_publisher_node'],
            name='robot_publisher_node',
            output='screen',
            shell=True
        ),

        # Start the FastAPI backend as a separate process
        ExecuteProcess(
            cmd=['python3', lib_directory],
            name='fastapi_server',
            output='screen',
            shell=True
        ),

        # Log that the launch file is executed
        LogInfo(msg="Launching ROS 2 Node and FastAPI server..."),
    ])
