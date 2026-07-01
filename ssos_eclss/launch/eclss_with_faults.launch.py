"""Launch the full ECLSS suite plus the fault-definitions config exposed as a
launch argument, so the simulation controller can schedule faults against the
ECLSS subsystems via /sim/inject_fault.

This reuses eclss.launch.py for the subsystem nodes and additionally publishes
the path to fault_definitions.yaml as a parameter for tooling/inspection."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ssos_eclss')
    faults_file = os.path.join(share, 'config', 'fault_definitions.yaml')

    fault_arg = DeclareLaunchArgument(
        'fault_definitions', default_value=faults_file,
        description='Path to the ECLSS fault-definitions YAML')

    eclss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'eclss.launch.py')))

    return LaunchDescription([
        fault_arg,
        LogInfo(msg=['ECLSS with faults; definitions: ',
                     LaunchConfiguration('fault_definitions')]),
        eclss,
    ])
