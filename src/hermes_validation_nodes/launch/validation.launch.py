from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hermes_validation_nodes",
            executable="hermes_validation_nodes",
            name="absolute_position_from_gazebo",
            parameters=[
                {"ros_pos_array_topic": "/hermes/gazebo/pose"},
                {"position_topic_out": "/robot/position/absolute"},
                {"position_value_index": 2}  # This parameter is optional and can be used for additional functionality
            ]
        )
    ])