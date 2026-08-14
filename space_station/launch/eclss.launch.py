"""Launch the high-fidelity ssos_eclss subsystems for the mission-control GUI.

This replaces the legacy space_station_eclss nodes with the new ssos_eclss
package (4BMS ARS, OGS, WRS, cabin) whose lifecycle nodes auto-configure and
activate and publish the /ssos/* telemetry the GUI ECLSS panel subscribes to
(/ssos/cabin/co2_ppm, /ssos/ars/co2_removal_kg_day, /ssos/ars/diagnostics,
/ssos/ars/bed_states, /ssos/ars/cycle_phase).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ssos_eclss_share = get_package_share_directory('ssos_eclss')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ssos_eclss_share, 'launch', 'eclss.launch.py')
            ),
        ),
    ])
