from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    space_station_pkg = get_package_share_directory('space_station')
    gnc_pkg = get_package_share_directory('space_station_gnc')

    # GNC + orbit dynamics drive the GNC panel (Earth + revolving ISS). Launched
    # quietly: gnc_core supports quiet_* args (output -> log, warn level), and
    # the orbit dynamics nodes are added here with output='log' so neither spams
    # the console.
    orbit_config = os.path.join(gnc_pkg, 'config', 'orbit_dynamics.yaml')

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
            executable='solar_power',
            name='solar_power_node',
            output='screen'
        ),

        # ---- GNC core (log-only, no console output) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gnc_pkg, 'launch', 'gnc_core.launch.py')
            ),
            launch_arguments={
                'quiet_torque': 'true',
                'quiet_control': 'true',
                'quiet_motion': 'true',
                'quiet_sensor': 'true',
                'quiet_estimate': 'true',
            }.items(),
        ),

        # ---- Orbit dynamics (log-only) ----
        Node(
            package='space_station_gnc',
            executable='orbit_dynamics',
            name='orbit_dynamics_node',
            output='log',
            parameters=[orbit_config],
        ),
        Node(
            package='space_station_gnc',
            executable='orbit_dynamics_mock',
            name='orbit_dynamics_mock',
            output='log',
        ),

    ])
