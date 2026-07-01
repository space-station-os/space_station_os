"""Launch just the WRS lifecycle node, auto-configured and activated."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ssos_eclss'), 'config', 'wrs_parameters.yaml')

    wrs = LifecycleNode(
        package='ssos_eclss', executable='wrs_node', name='wrs_node',
        namespace='', output='screen', parameters=[config])

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(wrs),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(wrs),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

    return LaunchDescription([
        wrs,
        RegisterEventHandler(OnProcessStart(
            target_action=wrs,
            on_start=[LogInfo(msg='wrs_node started; configuring...'), configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=wrs, goal_state='inactive',
            entities=[LogInfo(msg='wrs_node configured; activating...'), activate])),
    ])
