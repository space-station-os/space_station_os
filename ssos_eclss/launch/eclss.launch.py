"""Launch the full ECLSS suite (ARS, OGS, WRS, cabin) as lifecycle nodes.

All nodes are configured a couple of seconds in and activated shortly after via
TimerActions (scoped per node with matches_action). This deterministic, time-
based sequencing avoids the OnProcessStart/OnStateTransition event races that
could otherwise leave a node (e.g. wrs) stuck unconfigured when several
lifecycle nodes start together under one launch.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, TimerAction
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def _lifecycle(pkg_share, executable, name, config_file):
    config = os.path.join(pkg_share, 'config', config_file)
    return LifecycleNode(
        package='ssos_eclss', executable=executable, name=name,
        namespace='', output='screen', parameters=[config])


def _configure(node):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))


def _activate(node):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))


def generate_launch_description():
    share = get_package_share_directory('ssos_eclss')

    ars = _lifecycle(share, 'ars_node', 'ars_node', 'ars_parameters.yaml')
    ogs = _lifecycle(share, 'ogs_node', 'ogs_node', 'ogs_parameters.yaml')
    wrs = _lifecycle(share, 'wrs_node', 'wrs_node', 'wrs_parameters.yaml')
    cabin = _lifecycle(share, 'cabin_node', 'cabin_node', 'cabin_parameters.yaml')
    nodes = [ars, ogs, wrs, cabin]

    configure_all = TimerAction(
        period=2.0,
        actions=[LogInfo(msg='ECLSS: configuring ARS/OGS/WRS/cabin...')] +
                [_configure(n) for n in nodes])
    activate_all = TimerAction(
        period=4.0,
        actions=[LogInfo(msg='ECLSS: activating ARS/OGS/WRS/cabin...')] +
                [_activate(n) for n in nodes])

    return LaunchDescription([ars, ogs, wrs, cabin, configure_all, activate_all])
