"""Launch the full ECLSS suite (ARS, OGS, WRS, cabin).

Each node self-activates via its `autostart` parameter (set true here): it
configures then activates itself `autostart_delay_ms` after startup. This emits
no lifecycle ChangeState events from the launch (which are racy and produce
redundant 'transition not registered' errors when several managed nodes start
together). The delay is exposed as a launch argument so a parent launch can hold
ECLSS activation until after other systems (e.g. system_manager) are up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ssos_eclss')
    delay = ParameterValue(LaunchConfiguration('autostart_delay_ms'), value_type=int)

    def node(executable, name, config_file):
        return LifecycleNode(
            package='ssos_eclss', executable=executable, name=name,
            namespace='', output='screen',
            parameters=[os.path.join(share, 'config', config_file),
                        {'autostart': True, 'autostart_delay_ms': delay}])

    return LaunchDescription([
        DeclareLaunchArgument('autostart_delay_ms', default_value='300',
                              description='ms after startup before ECLSS nodes '
                                          'self-configure+activate'),
        node('ars_node', 'ars_node', 'ars_parameters.yaml'),
        node('ogs_node', 'ogs_node', 'ogs_parameters.yaml'),
        node('wrs_node', 'wrs_node', 'wrs_parameters.yaml'),
        node('cabin_node', 'cabin_node', 'cabin_parameters.yaml'),
        node('sabatier_node', 'sabatier_node', 'sabatier_parameters.yaml'),
        node('crew_node', 'crew_node', 'crew_parameters.yaml'),
    ])
