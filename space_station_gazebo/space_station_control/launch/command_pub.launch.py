from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('space_station_control'),
        'config',
        'ars_sys.yaml'
    )

    return LaunchDescription([
        Node(
            package='space_station_control',
            executable='teleop',
            name='operator',
            output='screen',
            # parameters=[params_file],
            emulate_tty=True
        ),

        Node(
            package='space_station_control',
            executable='mux',
            name='teleop_mux',
            output='screen',
            # parameters=[params_file],
            emulate_tty=True
        ),
        
    ])
