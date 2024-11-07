import launch
from launch import LaunchDescription
from rclpy.node import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hermes_robot',  # Replace with the name of your package
            executable='robot_node',      # Replace with the executable name (from the ROS 2 package)
            name='robot_node',            # Optional: name for the node (useful if running multiple instances)
            output='screen',              # Output the logs to the screen
        ),
    ])
    