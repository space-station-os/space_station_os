"""Launch just the OGS lifecycle node, auto-configured and activated."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ssos_eclss'), 'config', 'ogs_parameters.yaml')

    ogs = LifecycleNode(
        package='ssos_eclss', executable='ogs_node', name='ogs_node',
        namespace='', output='screen', parameters=[config])

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda node: True,
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))
    activate = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=lambda node: True,
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

    return LaunchDescription([
        ogs,
        RegisterEventHandler(OnProcessStart(
            target_action=ogs,
            on_start=[LogInfo(msg='ogs_node started; configuring...'), configure])),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=ogs, goal_state='inactive',
            entities=[LogInfo(msg='ogs_node configured; activating...'), activate])),
    ])
