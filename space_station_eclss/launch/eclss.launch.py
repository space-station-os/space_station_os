from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    CrewQuarterConfig = os.path.join(
        get_package_share_directory('space_station_eclss'),
        'config',
        'CrewQuarter.yaml'
    )
    Ars_params_file = os.path.join(
        get_package_share_directory('space_station_eclss'),
        'config',
        'ARS.yaml'
    )
    Wrs_params_file = os.path.join(
        get_package_share_directory('space_station_eclss'),
        'config',
        'WRS.yaml'
    )
    Ogs_params_file = os.path.join(
        get_package_share_directory('space_station_eclss'),
        'config',
        'OGS.yaml'
    )

    crew_quarters_node = Node(
        package='space_station_eclss',
        executable='crew_quarters',
        name='crew_quarters_node',
        output='screen',
        parameters=[CrewQuarterConfig],
        emulate_tty=True
    )

    ars_node = Node(
        package='space_station_eclss',
        executable='ars',
        name='air_revitalisation',
        output='screen',
        parameters=[Ars_params_file],
        emulate_tty=True
    )

    wrs_node = Node(
        package='space_station_eclss',
        executable='wrs',
        name='water_recovery_system',
        output='screen',
        parameters=[Wrs_params_file],
        emulate_tty=True
    )

    ogs_node = Node(
        package='space_station_eclss',
        executable='ogs',
        name='oxygen_generation_system',
        output='screen',
        parameters=[Ogs_params_file],
        emulate_tty=True
    )

    return LaunchDescription([
        ars_node,
        ogs_node,
        wrs_node,
        crew_quarters_node
    ])
