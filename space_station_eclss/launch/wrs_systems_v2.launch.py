from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        

        Node(
            package='space_station_eclss',
            executable='waste_collector',
            name='waste_collector',
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package='space_station_eclss',
        #     executable='ultrasound',
        #     name='ultrasound',
        #     output='screen',
        #     emulate_tty=True
        # ),
        Node(
            package='space_station_eclss',
            executable='upa',
            name='upa',
            output='screen',
            emulate_tty=True
        ),
        
        Node(
            package='space_station_eclss',
            executable='filter',
            name='filter',
            output='screen',
            emulate_tty=True
        ),
        
        Node(
            package='space_station_eclss',
            executable='catalytic_chamber',
            name='catalytic_chamber',
            output='screen',
            emulate_tty=True
        ),

        Node(
            package='space_station_eclss',
            executable='ionizaton',
            name='ionization',
            output='screen',
            emulate_tty=True
        ),
        
        Node(
            package='space_station_eclss',
            executable='clean_water_tank',
            name='clean_water_tank',
            output='screen',
            emulate_tty=True
        ),
    ])
