# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, AppendEnvironmentVariable

from launch_ros.actions import Node


def generate_launch_description():
    
    
    # Configure ROS nodes for launch

    # Setup project paths
    bridge_name_launch_config = LaunchConfiguration('bridge_name', default='ros_gz_bridge')
    pkg_project_description = get_package_share_directory('hermes_robot_description')
    
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    # Load the SDF file from "description" package
    urdf_file  =  os.path.join(pkg_project_description, 'urdf', 'hermes_robot_2.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # For publishing and controlling the robot pose, we need joint states of the robot
    # Configure the robot model by adjusting the joint angles using the GUI slider
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_file],
        output=['screen']
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project_description, 'config', 'config.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Bridge Node
    # ros_gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     # name=bridge_name_launch_config,
    #     output='screen',
    #     parameters=[{"config_file": "/Robots/hermes_robot_description/config/topic_mapping.yaml"}]
    # )
    
    # Include the Gazebo simulator launch
    # include_gz_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gz_launch_path)
    # )
    
    # # Setup to launch the simulator and Gazebo world
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': PathJoinSubstitution([
    #         pkg_project_description,
    #         'urdf',
    #         'hermes_robot_description_2.urdf'
    #     ])}.items(),
    # )
    
    
    # Node to spawn the robot in Gazebo
    # spawn_robot = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '-file', urdf_file,
    #         '-name', 'hermes_robot_description',
    #         '-x', '0', '-y', '0', '-z', '1'
    #     ],
    #     output='screen'
    # )
    

    ld = LaunchDescription([
        # gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        # joint_state_publisher_gui,
        robot_state_publisher,
        rviz
    ])
    
    # ld.add_action(ros_gz_bridge)
    # ld.add_action(include_gz_sim_launch)

    # ld.add_action(spawn_robot)
    return ld
    
    