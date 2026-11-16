"""Launch the ssos_eps generation and storage node.

The node self-activates via its `autostart` parameter: it configures then
activates itself `autostart_delay_ms` after startup, so no lifecycle
ChangeState events are emitted from the launch. A parent launch can raise the
delay to hold EPS until system_manager and simulation_controller are active.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ssos_eps')
    delay = ParameterValue(LaunchConfiguration('autostart_delay_ms'), value_type=int)

    def config(name):
        return os.path.join(share, 'config', name)

    eps = LifecycleNode(
        package='ssos_eps', executable='eps_node', name='eps_node',
        namespace='', output='screen',
        parameters=[config('eps_parameters.yaml'),
                    config('solar_parameters.yaml'),
                    config('battery_parameters.yaml'),
                    config('bcdu_parameters.yaml'),
                    {'autostart': True, 'autostart_delay_ms': delay}])

    return LaunchDescription([
        DeclareLaunchArgument('autostart_delay_ms', default_value='300',
                              description='ms after startup before eps_node '
                                          'self-configures+activates'),
        eps,
    ])
