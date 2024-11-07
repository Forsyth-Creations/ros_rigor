from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, SetEnvironmentVariable, 
    IncludeLaunchDescription, SetLaunchConfiguration
)
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

import os

urdf_tutorial_launch_path = PathJoinSubstitution(
    [FindPackageShare("urdf_tutorial"), "launch", "display.launch.py"]
)

def generate_launch_description():
    pkg_project_bringup = get_package_share_directory('hermes_robot_description')
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    bridge_name_launch_config = LaunchConfiguration('bridge_name', default='ros_gz_bridge')
    config_file_launch_config = LaunchConfiguration('config_file', default='/Robots/hermes_robot_description/config/topic_mapping.yaml')

    declare_bridge_name_cmd = DeclareLaunchArgument(
        'bridge_name', description='Name of ros_gz_bridge node', default_value=bridge_name_launch_config
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', description='YAML config file', default_value=config_file_launch_config
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='moon',
        choices=['moon', 'mars', 'enceladus'],
        description='World to load into Gazebo'
    )

    # Define world file configuration based on selected world
    SetLaunchConfiguration(
        name='world_file', 
        value=[LaunchConfiguration('world'), TextSubstitution(text='.sdf')]
    )

    # Include the Gazebo simulator launch
    include_gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path)
    )

    # Bridge node to connect ROS and Gazebo
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=bridge_name_launch_config,
        output='screen',
        parameters=[{"config_file": "/Robots/hermes_robot_description/config/topic_mapping.yaml"}]
    )

    # Define robot description for Robot State Publisher
    pkg_project_description = get_package_share_directory('hermes_robot_description')
    urdf_file = os.path.join(pkg_project_description, 'urdf', 'hermes_robot_description_2.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[{'use_sim_time': True}, {'robot_description': robot_desc}]
    # )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_file],
        output='screen'
    )

    # Node to spawn the robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'hermes_robot_description',
            '-x', '0', '-y', '0', '-z', '2'
        ],
        output='screen'
    )
    
    # ros2 launch urdf_tutorial display.launch.py model:=/Robots/hermes_robot_description/urdf/hermes_robot_description.urdf
    # Launch Description
    # TODO
    rviz_node = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(urdf_tutorial_launch_path),
    launch_arguments={'model': '/Robots/hermes_robot_description/urdf/hermes_robot_description.urdf'}.items(),
    )
    
    # rviz = Node(
    #    package='rviz2',
    #    executable='rviz2',
    # #    arguments=['-d', os.path.join(pkg_project_bringup, 'config', 'rrbot.rviz')],
    # #    condition=IfCondition(LaunchConfiguration('rviz'))
    # )
    
    
    ld = LaunchDescription()

    # Add actions
    ld.add_action(declare_bridge_name_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(include_gz_sim_launch)
    ld.add_action(ros_gz_bridge)
    # ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(spawn_robot)  # Add the spawn action here to spawn the robot in Gazebo
    # ld.add_action(rviz)
    ld.add_action(rviz_node)  # Add RViz node here

    return ld
