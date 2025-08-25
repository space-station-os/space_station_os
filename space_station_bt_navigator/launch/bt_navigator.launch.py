#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_failsafe_cmd = DeclareLaunchArgument(
        'failsafe',
        default_value='false',
        description='Enable failsafe mode for the behavior tree'
    )
    
    declare_plugin_libraries_cmd = DeclareLaunchArgument(
        'plugin_libraries',
        default_value='space_station_bt_nodes',
        description='Comma-separated list of plugin libraries to load'
    )
    
    declare_tree_xml_cmd = DeclareLaunchArgument(
        'tree_xml',
        default_value='failsafe_tree.xml',
        description='XML file defining the behavior tree'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    bt_navigator_node = LifecycleNode(
        package='space_station_bt_navigator',
        executable='bt_navigator_node',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'failsafe': LaunchConfiguration('failsafe'),
            'plugin_libraries': LaunchConfiguration('plugin_libraries'),
            'tree_xml': LaunchConfiguration('tree_xml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('gnc/unload_cmg', 'gnc/unload_cmg'),
        ]
    )

    configure_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_state_machine=bt_navigator_node,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )),
            ],
        ),
    )

    activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_state='configured',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_state_machine=bt_navigator_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
    )

    return LaunchDescription([
        declare_failsafe_cmd,
        declare_plugin_libraries_cmd,
        declare_tree_xml_cmd,
        declare_use_sim_time_cmd,
        bt_navigator_node,
        configure_event_handler,
        activate_event_handler,
    ])
