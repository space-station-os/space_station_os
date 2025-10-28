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

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    orbital_config_file = os.path.join(
        get_package_share_directory('space_station_gnc'),
        'config',
        'orbit_dynamics.yaml'
    )
    return LaunchDescription([


        Node(
            package='space_station_gnc',
            executable='orbit_dynamics',
            name='orbit_dynamics_node',
            output='screen',
            parameters=[orbital_config_file],
            emulate_tty=True
        ),
        Node(
            package='space_station_gnc',
            executable='orbit_dynamics_mock',
            name='orbit_dynamics_mock',
            output='screen',

            emulate_tty=True
        ),


    ])
