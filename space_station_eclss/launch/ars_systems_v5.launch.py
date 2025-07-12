from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('space_station_eclss'),
        'config',
        'ARS.yaml'
    )

    return LaunchDescription([
        Node(
            package='space_station_eclss',
            executable='ars',
            name='air_revitalisation',
            output='screen',
            parameters=[params_file],
            emulate_tty=True
        ),

        # Node(
        #     package='space_station_eclss',
        #     executable='desiccant1',
        #     name='desiccant_bed_1',
        #     output='screen',
        #     parameters=[params_file],
        #     emulate_tty=True
        # ),
        # Node(
        #     package='space_station_eclss',
        #     executable='desiccant2',
        #     name='desiccant_bed_2',
        #     output='screen',
        #     parameters=[params_file],
        #     emulate_tty=True
        # ),
        # Node(
        #     package='space_station_eclss',
        #     executable='adsorbent1',
        #     name='adsorbent_bed_1',
        #     output='screen',
        #     parameters=[params_file],
        #     emulate_tty=True
        # ),
        # Node(
        #     package='space_station_eclss',
        #     executable='adsorbent2',
        #     name='adsorbent_bed_2',
        #     output='screen',
        #     parameters=[params_file],
        #     emulate_tty=True
        # ),

        # Node(
        #     package='space_station_eclss',
        #     executable='safety',
        #     name='stl_monitor',
        #     output='screen',
        #     parameters=[params_file],
        #     emulate_tty=True
        # ),
       
       
    ])
