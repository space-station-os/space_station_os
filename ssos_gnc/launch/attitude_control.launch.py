#!/usr/bin/env python3
#
# Copyright 2026 Space Station OS
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    share = get_package_share_directory('ssos_gnc')
    control_params = os.path.join(share, 'config', 'attitude_control.yaml')
    plant_params = os.path.join(share, 'config', 'gnc_plant.yaml')

    autostart = LaunchConfiguration('autostart')

    plant = LifecycleNode(
        package='ssos_gnc',
        executable='gnc_plant_node',
        name='gnc_plant',
        namespace='',
        output='screen',
        parameters=[plant_params],
    )

    control = LifecycleNode(
        package='ssos_gnc',
        executable='attitude_control_node',
        name='attitude_control',
        namespace='',
        output='screen',
        parameters=[control_params],
    )

    def lifecycle_calls(node_name, delay):
        return TimerAction(
            period=delay,
            condition=IfCondition(autostart),
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'configure'],
                    output='screen',
                ),
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=['ros2', 'lifecycle', 'set', f'/{node_name}', 'activate'],
                            output='screen',
                        )
                    ],
                ),
            ],
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Drive both lifecycle nodes to active on launch.',
        ),
        plant,
        control,
        lifecycle_calls('gnc_plant', 2.0),
        lifecycle_calls('attitude_control', 5.0),
    ])
