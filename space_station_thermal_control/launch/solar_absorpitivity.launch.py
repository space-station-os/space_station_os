from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('space_station_thermal_control'),
        'config',
        'solar_array.yaml'
    )

    return LaunchDescription([
        # Sun vector provider node
        Node(
            package='space_station_thermal_control',
            executable='sun_vector',
            name='sun_vector',
            output='screen',
            emulate_tty=True
        ),

        # Solar panel absorptivity heat calculation
        Node(
            package='space_station_thermal_control',
            executable='array_absorptivity',
            name='solar_heat_node',  
            output='screen',
            parameters=[config_file],
            emulate_tty=True
        ),
    ])
