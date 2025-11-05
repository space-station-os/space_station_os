from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    pkg_dir = get_package_share_directory('space_station_communication')

    apid_config_path = os.path.join(pkg_dir, 'config', 'bridge.yaml')
    relay_script_path = os.path.join(pkg_dir, 'starlink', 'starlink_relay.py')

    return LaunchDescription([
        # Launch the ROS 2 bridge node
        Node(
            package='space_station_communication',
            executable='space_bridge',
            name='ros2_ccsds_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'apid_config': apid_config_path,
            }]
        ),

        # Launch the Starlink relay server (Python script)
        ExecuteProcess(
            cmd=['python3', relay_script_path],
            output='screen',
        ),
    ])
