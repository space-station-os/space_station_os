"""Launch the full ECLSS suite (ARS, OGS, WRS, cabin) as lifecycle nodes,
each auto-configured and activated from its config file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def _lifecycle(pkg_share, executable, name, config_file):
    config = os.path.join(pkg_share, 'config', config_file)
    return LifecycleNode(
        package='ssos_eclss', executable=executable, name=name,
        namespace='', output='screen', parameters=[config])


def _auto_manage(node, label):
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: True,
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda n: True,
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))
    return [
        RegisterEventHandler(OnProcessStart(
            target_action=node,
            on_start=[LogInfo(msg=f'{label} started; configuring...'), configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=node, goal_state='inactive',
            entities=[LogInfo(msg=f'{label} configured; activating...'), activate])),
    ]


def generate_launch_description():
    share = get_package_share_directory('ssos_eclss')

    ars = _lifecycle(share, 'ars_node', 'ars_node', 'ars_parameters.yaml')
    ogs = _lifecycle(share, 'ogs_node', 'ogs_node', 'ogs_parameters.yaml')
    wrs = _lifecycle(share, 'wrs_node', 'wrs_node', 'wrs_parameters.yaml')
    cabin = _lifecycle(share, 'cabin_node', 'cabin_node', 'cabin_parameters.yaml')

    ld = LaunchDescription([ars, ogs, wrs, cabin])
    for node, label in [(ars, 'ars_node'), (ogs, 'ogs_node'),
                        (wrs, 'wrs_node'), (cabin, 'cabin_node')]:
        for handler in _auto_manage(node, label):
            ld.add_action(handler)
    return ld
