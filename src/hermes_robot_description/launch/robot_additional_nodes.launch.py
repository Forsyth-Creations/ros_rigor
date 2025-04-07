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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    ld = LaunchDescription()

    depth_to_laser_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'depth.launch.py'
    )
    
    
    depth_to_laser_node =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_to_laser_launch_path)
    )
    
    # Launch SLAM
    
    depth_to_laser_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'slam_online_async_launch.py'
    )
    
    slam_toolbox_node =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_to_laser_launch_path)
    )
    
    # Launch Nav2
    
    nav2_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'nav2.launch.py'
    )
    
    nav2_node =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path)
    )
    
    twist_mux_launch_path = os.path.join(
        get_package_share_directory('hermes_robot_description'), 
        'launch', 
        'mux.launch.py'
    )
    
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_mux_launch_path)
    )
    
    
    # ld.add_action(twist_mux)
    ld.add_action(depth_to_laser_node)
    ld.add_action(slam_toolbox_node)
    ld.add_action(nav2_node)
    
    return ld
