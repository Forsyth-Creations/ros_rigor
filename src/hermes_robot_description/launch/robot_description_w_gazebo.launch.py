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
import subprocess

def generate_launch_description():
    pkg_hermes_robot_description = get_package_share_directory('hermes_robot_description')
    
    # Convert hte corrected.urdf.xacro file to a urdf file
    xacro_path = os.path.join(pkg_hermes_robot_description, 'urdf', 'corrected2.urdf.xacro')
    
    # Convert the xacro file to a urdf file
    result = subprocess.run(['xacro', xacro_path], capture_output=True, text=True)
    urdf_xml = result.stdout
    
    # Save the urdf to a file
    urdf_path = os.path.join(pkg_hermes_robot_description, 'urdf', 'corrected.urdf')
    with open(urdf_path, 'w') as outfp:
        outfp.write(urdf_xml)
    
        
    # Load the URDF into RVIZ
    # urdf_path = os.path.join(pkg_hermes_robot_description, 'urdf', 'corrected.urdf')
    # with open(urdf_path, 'r') as infp:
    #     urdf_xml = infp.read()
        
    # Convert the urdf to sdf using the urdf_to_sdf tool
    # Convert URDF to SDF
    result = subprocess.run(['gz', 'sdf', '-p', urdf_path], capture_output=True, text=True)
    sdf_content = result.stdout
    
    # Write the sdf to a file
    sdf_path = os.path.join(pkg_hermes_robot_description, 'urdf', 'corrected.sdf')
    with open(sdf_path, 'w') as outfp:
        outfp.write(sdf_content)
        
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            sdf_content,
            "-name",
            "hermes_robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.0",
            "-Y",
            "1.5708",  # 90 degrees in radians
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
