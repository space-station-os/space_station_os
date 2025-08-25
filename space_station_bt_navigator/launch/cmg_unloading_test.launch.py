#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_cmg_threshold_x_cmd = DeclareLaunchArgument(
        'cmg_threshold_x',
        default_value='100.0',
        description='X-axis CMG threshold (Nms)'
    )
    
    declare_cmg_threshold_y_cmd = DeclareLaunchArgument(
        'cmg_threshold_y',
        default_value='100.0',
        description='Y-axis CMG threshold (Nms)'
    )
    
    declare_cmg_threshold_z_cmd = DeclareLaunchArgument(
        'cmg_threshold_z',
        default_value='100.0',
        description='Z-axis CMG threshold (Nms)'
    )

    bt_launcher_node = Node(
        package='space_station_bt_navigator',
        executable='bt_navigator_node',
        name='cmg_unloading_bt',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'plugin_libraries': 'space_station_bt_nodes',
            'tree_xml': 'cmg_unloading_tree.xml',
            'cmg_threshold_x': LaunchConfiguration('cmg_threshold_x'),
            'cmg_threshold_y': LaunchConfiguration('cmg_threshold_y'),
            'cmg_threshold_z': LaunchConfiguration('cmg_threshold_z'),
        }],
        remappings=[
            ('gnc/unload_cmg', 'gnc/unload_cmg'),
            ('gnc/cmg_h', 'gnc/cmg_h'),
            ('gnc/cmg_status', 'gnc/cmg_status'),
        ]
    )

    cmg_simulator_node = Node(
        package='space_station_bt_nodes',
        executable='cmg_simulator',
        name='cmg_simulator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_rate': 10.0,
            'initial_cmg_h_x': 0.0,
            'initial_cmg_h_y': 0.0,
            'initial_cmg_h_z': 0.0,
        }],
        remappings=[
            ('cmg_h', 'gnc/cmg_h'),
            ('cmg_status', 'gnc/cmg_status'),
        ]
    )

    log_info_cmd = LogInfo(
        msg="Launching CMG Unloading Behavior Tree Test with thresholds: " +
            "X={}, Y={}, Z={}".format(
                LaunchConfiguration('cmg_threshold_x'),
                LaunchConfiguration('cmg_threshold_y'),
                LaunchConfiguration('cmg_threshold_z')
            )
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_cmg_threshold_x_cmd,
        declare_cmg_threshold_y_cmd,
        declare_cmg_threshold_z_cmd,
        log_info_cmd,
        bt_launcher_node,
        cmg_simulator_node,
    ])
