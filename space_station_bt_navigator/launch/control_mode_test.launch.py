#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gnc_pkg_dir = get_package_share_directory('space_station_gnc')
    bt_navigator_pkg_dir = get_package_share_directory('space_station_bt_navigator')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    control_mode_manager_node = Node(
        package='space_station_gnc',
        executable='control_mode_manager',
        name='control_mode_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    torque_controller_node = Node(
        package='space_station_gnc',
        executable='torque_controller',
        name='torque_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    bt_navigator_node = Node(
        package='space_station_bt_navigator',
        executable='bt_navigator_node',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tree_xml': PathJoinSubstitution([bt_navigator_pkg_dir, 'behavior_trees', 'control_mode_test_tree.xml'])
        }]
    )
    
    ld = LaunchDescription()
    
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(control_mode_manager_node)
    ld.add_action(torque_controller_node)
    ld.add_action(bt_navigator_node)
    
    return ld
