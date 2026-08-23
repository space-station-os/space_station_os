"""Launch the ssos_thermal nodes: thermal_network, coolant, sun_vector,
solar_heat.

thermal_network and coolant self-activate via their `autostart` parameter
(set true here): each configures then activates itself
`autostart_delay_ms` after startup. This emits no lifecycle ChangeState
events from the launch (which are racy and produce redundant 'transition
not registered' errors when several managed nodes start together),
mirroring ssos_eclss/launch/eclss.launch.py. The delay is exposed as a
launch argument so a parent launch can hold thermal activation until after
other systems (e.g. system_manager) are up.

coolant hosts the /coolant_heat_transfer action that both
thermal_network's cooling client and the mission-control GUI's
ThermalWidget (Internal/Ammonia Temp cards) consume -- without it running,
those cards stay "NO DATA".
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ssos_thermal')
    delay = ParameterValue(LaunchConfiguration('autostart_delay_ms'), value_type=int)

    thermal_network = LifecycleNode(
        package='ssos_thermal',
        executable='thermal_network_node',
        name='thermal_network',
        namespace='',
        output='screen',
        parameters=[os.path.join(share, 'config', 'thermal_network.yaml'),
                    {'autostart': True, 'autostart_delay_ms': delay}])

    coolant = LifecycleNode(
        package='ssos_thermal',
        executable='coolant_node',
        name='coolant_node',
        namespace='',
        output='screen',
        parameters=[os.path.join(share, 'config', 'coolant.yaml'),
                    {'autostart': True, 'autostart_delay_ms': delay}])

    sun_vector = Node(
        package='ssos_thermal',
        executable='sun_vector_node',
        name='sun_vector_node',
        output='screen')

    solar_heat = Node(
        package='ssos_thermal',
        executable='solar_heat_node',
        name='solar_heat_node',
        output='screen')

    visualization = Node(
        package='ssos_thermal',
        executable='thermal_visualization.py',
        name='ssos_thermal_visualization',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('launch_visualization')))

    return LaunchDescription([
        DeclareLaunchArgument('autostart_delay_ms', default_value='300',
                              description='ms after startup before thermal_network '
                                          'self-configures+activates'),
        DeclareLaunchArgument('launch_visualization', default_value='true',
                              description='Launch the standalone PyQt5 thermal '
                                          'viewer. Set false when included from a '
                                          'parent launch (e.g. space_station.launch.py) '
                                          'whose own GUI already has a Thermal panel.'),
        thermal_network,
        coolant,
        sun_vector,
        solar_heat,
        visualization,
    ])
