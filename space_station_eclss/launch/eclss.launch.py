from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    CrewQuarterConfig = os.path.join(
        get_package_share_directory('space_station_eclss'),
        'config',
        'CrewQuarter.yaml'
    )
    
    crew_quarters_node=Node(
        package='space_station_eclss',
        executable='crew_quarters',
        name='crew_quarters_node',
        output='screen',
        parameters=[CrewQuarterConfig],
        emulate_tty=True)
    
    ars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('space_station_eclss'), 'launch', 'ars_systems_v5.launch.py')
        )
    )

    ogs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('space_station_eclss'), 'launch', 'ogs_systems_v3.launch.py')
        )
    )

    wrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('space_station_eclss'), 'launch', 'wrs_systems_v3.launch.py')
        )
    )

    return LaunchDescription([
        ars_launch,
        ogs_launch,
        wrs_launch,
        crew_quarters_node
    ])
