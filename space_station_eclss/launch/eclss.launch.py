from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('space_station_eclss'), 'launch', 'ars_systems_v4.launch.py')
        )
    )

    ogs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('space_station_eclss'), 'launch', 'ogs_systems_v2.launch.py')
        )
    )

    wrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('space_station_eclss'), 'launch', 'wrs_systems.launch.py')
        )
    )

    return LaunchDescription([
        ars_launch,
        ogs_launch,
        wrs_launch,
    ])
