"""Launch the ssos_eps generation and storage node for the mission-control GUI.

This replaces the legacy space_station_eps nodes with the ssos_eps lifecycle
node, which auto-configures and activates and publishes the /ssos/eps/*
telemetry the GUI EPS panel subscribes to (/ssos/eps/diagnostics,
/ssos/eps/bcdu/status, /ssos/eps/bcdu/channel_<n>/status,
/ssos/eps/battery/oru_<n>/state).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ssos_eps_share = get_package_share_directory('ssos_eps')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ssos_eps_share, 'launch', 'eps.launch.py')
            ),
        ),
    ])
