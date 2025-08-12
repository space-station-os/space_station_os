from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    apid_config_path = os.path.join(
        get_package_share_directory('demo_ros_ccsds_bridge'),
        'config',
        'bridge.yaml'
    )

    return LaunchDescription([
        Node(
            package='demo_ros_ccsds_bridge',
            executable='ground_receiver',
            name='ros2_ccsds_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'apid_config': apid_config_path,
                
            }]
        )
    ])
