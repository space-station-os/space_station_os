"""Launch the full ECLSS suite (ARS, OGS, WRS, cabin).

Each node self-activates via its `autostart` parameter (set true here): it
configures then activates itself a fraction of a second after startup. This
avoids emitting any lifecycle ChangeState events from the launch, which are
racy and produce redundant 'transition not registered' errors when several
managed nodes start together. For externally-managed lifecycle (e.g. the
full-system launch with a system_manager), leave autostart false and drive the
transitions explicitly instead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def _node(pkg_share, executable, name, config_file):
    config = os.path.join(pkg_share, 'config', config_file)
    return LifecycleNode(
        package='ssos_eclss', executable=executable, name=name,
        namespace='', output='screen',
        parameters=[config, {'autostart': True}])


def generate_launch_description():
    share = get_package_share_directory('ssos_eclss')
    return LaunchDescription([
        _node(share, 'ars_node', 'ars_node', 'ars_parameters.yaml'),
        _node(share, 'ogs_node', 'ogs_node', 'ogs_parameters.yaml'),
        _node(share, 'wrs_node', 'wrs_node', 'wrs_parameters.yaml'),
        _node(share, 'cabin_node', 'cabin_node', 'cabin_parameters.yaml'),
    ])
