#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_share_directory = get_package_share_directory('space_station_gnc')
    # urdf_file_path = os.path.join(package_share_directory, 'urdf', 'ISS_stationaryFromNasa.urdf')
    urdf_file_path = os.path.join(package_share_directory, 'urdf', 'SD_SpaceStation_Ver05.urdf')
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
