from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    thermal_config_file = os.path.join(
        get_package_share_directory('space_station_gnc'),
        'config',
        'orbit_dynamics.yaml'
    )
    return LaunchDescription([
        

        Node(
            package='space_station_gnc',
            executable='orbit_dynamics',
            name='orbit_dynamics_node',
            output='screen',
            parameters=[thermal_config_file],
            emulate_tty=True
        ),

    ])
