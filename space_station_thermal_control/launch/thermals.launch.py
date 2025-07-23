from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('space_station_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'space_station.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    
    return LaunchDescription([
        

        Node(
            package='space_station_thermal_control',
            executable='coolant',
            name='internal_coolant',
            output='screen',
            
            emulate_tty=True
        ),

        Node(
            package='space_station_thermal_control',
            executable='external_loop',
            name='external_loop',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='space_station_thermal_control',
            executable='radiator',
            name='radiators',
            output='screen',
            emulate_tty=True
        ),
        
        Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
        ),
        
        Node(
            package='space_station_thermal_control',
            executable='thermal_nodes',
            name='thermal_network',
            output='screen',
            
            emulate_tty=True
        
    )
    ])
