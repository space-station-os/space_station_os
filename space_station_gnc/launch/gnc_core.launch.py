#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_nodes(context, *args, **kwargs):
    def output_for(flag_name):
        return 'log' if LaunchConfiguration(flag_name).perform(context) == 'true' else 'screen'

    def arguments_for(flag_name):
        return ['--ros-args', '--log-level', 'warn'] if LaunchConfiguration(flag_name).perform(context) == 'true' else []

    return [
        Node(
            package='space_station_gnc',
            executable='torque_collector',
            name='torque_collector',
            output=output_for('quiet_torque'),
            arguments=arguments_for('quiet_torque')
        ),
        Node(
            package='space_station_gnc',
            # executable='control_torque_act',
            # name='control_torque_act',
            executable='control_torque',
            name='control_torque',
            output=output_for('quiet_control'),
            arguments=arguments_for('quiet_control')
        ),
        Node(
            package='space_station_gnc',
            executable='physics_motion',
            name='physics_motion',
            output=output_for('quiet_motion'),
            arguments=arguments_for('quiet_motion')
        ),
        Node(
            package='space_station_gnc',
            executable='physics_sensor',
            name='physics_sensor',
            output=output_for('quiet_sensor'),
            arguments=arguments_for('quiet_sensor')
        ),
        Node(
            package='space_station_gnc',
            executable='sense_estimate',
            name='sense_estimate',
            output=output_for('quiet_estimate'),
            arguments=arguments_for('quiet_estimate')
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('quiet_torque', default_value='false'),
        DeclareLaunchArgument('quiet_control', default_value='false'),
        DeclareLaunchArgument('quiet_motion', default_value='false'),
        DeclareLaunchArgument('quiet_sensor', default_value='true'),
        DeclareLaunchArgument('quiet_estimate', default_value='false'),

        OpaqueFunction(function=launch_nodes)
    ])

