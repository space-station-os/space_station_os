from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
   
    space_station_pkg = get_package_share_directory('space_station')

    return LaunchDescription([
       
       
       Node(
            package='space_station',
            executable='space_station',
            name='space_station_gui_node',
            output='screen'
        ),
       
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(space_station_pkg, 'launch', 'eclss.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(space_station_pkg, 'launch', 'thermals.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(space_station_pkg, 'launch', 'eps.launch.py')
            ),
        ),

        Node(
            package='space_station_eps',
            executable= 'solar_power',
            name='solar_power_node',
            output='screen'
        )
        
        
    ])
