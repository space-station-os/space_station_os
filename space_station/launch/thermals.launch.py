from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    thermal_config_file = os.path.join(
        get_package_share_directory('space_station_thermal_control'),
        'config',
        'thermals.yaml'
    )
    return LaunchDescription([
    
        Node(
            package='space_station_thermal_control',
            executable='cooling_server',
            name='cooling_server',
            emulate_tty=True,
            output='screen'),
        
        Node(
            package='space_station_thermal_control',
            executable='radiator',
            name='radiators',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='space_station_thermal_control',
            executable='thermal_nodes',
            name='thermal_network',
            output='screen',
            parameters=[thermal_config_file],
            emulate_tty=True
        ),
        
        # Node(
        #     package='space_station_thermal_control',
        #     executable='thermal_visualization.py',
        #     name='thermal_visualization',
        #     output='screen',
        #     emulate_tty=True
        # )
    ])
