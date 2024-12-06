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
from launch_ros.actions import Node

def generate_launch_description():
    pkg_hermes_robot_description = get_package_share_directory('hermes_robot_description')
        
    # Load the URDF into RVIZ
    urdf_path = os.path.join(pkg_hermes_robot_description, 'urdf', 'hermes.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_xml = infp.read()
        
        
    # Get the file path of the SDF file
    sdf_path = os.path.join(pkg_hermes_robot_description, 'urdf', 'hermes.sdf')
    with open(sdf_path, 'r') as infp:
        sdf_xml = infp.read()

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            sdf_xml,
            "-name",
            "hermes_robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            ".4",
        ],
        output="screen",
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': urdf_xml}
        ]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    
    return ld
