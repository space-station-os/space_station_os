#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import xacro

def launch_setup(context, *args, **kwargs):
    use_fixed_joint = str(LaunchConfiguration("use_fixed_joint").perform(context))
    # print(f"[DEBUG] use_fixed_joint = {use_fixed_joint}") 
    share_dir = get_package_share_directory('space_station_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'space_station.xacro')

    robot_description_config = xacro.process_file(
        xacro_file,
        mappings={'use_fixed_joint': use_fixed_joint}
    )
    robot_urdf = robot_description_config.toxml()

    # with open('/tmp/robot.urdf', 'w') as f:
    #     f.write(robot_urdf)
    rviz_config_path = os.path.join(
        get_package_share_directory('space_station_gnc'),
        'rviz',
        'ssos.rviz'
    )

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_urdf}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path] 
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fixed_joint',
            default_value='true',
            description='Use fixed joints instead of continuous for tf broadcasting'
        ),
        OpaqueFunction(function=launch_setup)
    ])
