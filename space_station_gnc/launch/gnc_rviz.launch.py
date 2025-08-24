#!/usr/bin/env python3
#
# Copyright 2025 Space Station OS
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

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_directory = get_package_share_directory('space_station_gnc')
    urdf_file_path = os.path.join(package_share_directory, 'urdf', 'ISS_stationaryFromNasa.urdf')
    # urdf_file_path = os.path.join(package_share_directory, 'urdf', 'SD_SpaceStation_Ver05.urdf')
    rviz_config_path = os.path.join(package_share_directory, 'rviz', 'your_config_file.rviz')  # TODO

    with open(urdf_file_path, 'r') as urdf_file:
        urdf_data = urdf_file.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_data}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[] 
            # arguments=['-d', rviz_config_path] 
        )
    ])
