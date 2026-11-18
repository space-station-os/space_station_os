"""Launch ssos_thermal for the mission-control GUI.

This replaces the legacy space_station_thermal_control nodes with the
ssos_thermal lifecycle nodes (thermal_network and coolant). The full station
runs the station network (config/station_thermal_nodes.yaml), whose cold plate
and cabin nodes take heat from ssos_eps on /thermal/heat_loads. The GUI has its
own Thermal panel, so the standalone viewer is not started.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ssos_thermal_share = get_package_share_directory('ssos_thermal')
    return LaunchDescription([
        DeclareLaunchArgument('autostart_delay_ms', default_value='300'),
        DeclareLaunchArgument('thermal_config_file',
                              default_value='config/station_thermal_nodes.yaml'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ssos_thermal_share, 'launch', 'thermal.launch.py')
            ),
            launch_arguments={
                'autostart_delay_ms': LaunchConfiguration('autostart_delay_ms'),
                'thermal_config_file': LaunchConfiguration('thermal_config_file'),
                'launch_visualization': 'false',
            }.items(),
        ),
    ])
