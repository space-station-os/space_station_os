from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
            get_package_share_directory('space_station_eclss'),
            'config',
            'WRS.yaml'
        ),
    return LaunchDescription([    
        
        Node(
            package='space_station_eclss',
            executable='wrs',
            name='water_recovery_system',
            output='screen',
            parameters=[config_file],
            emulate_tty=True
        ),
        
    ])
