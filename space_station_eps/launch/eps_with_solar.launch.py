from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    primary_eps_config = os.path.join(
        get_package_share_directory('space_station_eps'),
        'config',
        'primary_eps_config.yaml'
    )

    solar_power = Node(
        package='space_station_eps',
        executable='solar_power',
        name='mock_solar_controller',
        output='screen',
        parameters=[primary_eps_config],
        emulate_tty=True,
    )

    battery_manager = Node(
        package='space_station_eps',
        executable='battery_manager_node',
        name='battery_manager',
        output='screen',
        parameters=[primary_eps_config],
        emulate_tty=True,
    )

    bcdu = Node(
        package='space_station_eps',
        executable='bcdu_node',
        name='bcdu_node',
        output='screen',
        parameters=[primary_eps_config],
        emulate_tty=True,
    )

    ddcu = Node(
        package='space_station_eps',
        executable='ddcu_device',
        name='ddcu_node',
        output='screen',
        parameters=[primary_eps_config],
        emulate_tty=True,
    )

    mbsu_device = Node(
        package='space_station_eps',
        executable='mbsu_device',
        name='mbsu_node',
        output='screen',
        parameters=[primary_eps_config],
        emulate_tty=True,
    )

    return LaunchDescription([
        solar_power,
        battery_manager,
        bcdu,
        ddcu,
        mbsu_device,
    ])
