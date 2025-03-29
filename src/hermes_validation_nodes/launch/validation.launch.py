from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    absolute_pos = Node(
        package="hermes_validation_nodes",
        executable="hermes_validation_nodes",
        name="absolute_position_from_gazebo",
        parameters=[
            {"ros_pos_array_topic": "/hermes/gazebo/pose"},
            {"position_topic_out": "/robot/position/absolute"},
            {
                "position_value_index": 2
            },  # This parameter is optional and can be used for additional functionality
            {"use_sim_time": True},
        ],
    )
    
    clamp = Node(
        package="hermes_validation_nodes",
        executable="hermes_validation_clamp",
        name="clamp",
        parameters=[
            {"input_topic": "/goal_pose"},
            {"output_topic": "/clamped_goal_pose"},
            {"message_type": "geometry_msgs.msg.PoseStamped"},
            {"publish_frequency": 30.0},
            {"use_sim_time": True},
        ],
    )

    ld.add_action(absolute_pos)
    ld.add_action(clamp)

    return ld
