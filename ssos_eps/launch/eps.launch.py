"""Launch the SSOS EPS lifecycle node and activate it automatically."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("ssos_eps"),
        "config",
        "eps_parameters.yaml",
    )

    eps = LifecycleNode(
        package="ssos_eps",
        executable="eps_node",
        name="eps_node",
        namespace="",
        output="screen",
        parameters=[config],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(eps),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(eps),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    return LaunchDescription(
        [
            eps,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=eps,
                    on_start=[
                        LogInfo(msg="eps_node started; configuring..."),
                        configure,
                    ],
                )
            ),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=eps,
                    goal_state="inactive",
                    entities=[
                        LogInfo(msg="eps_node configured; activating..."),
                        activate,
                    ],
                    handle_once=True,
                )
            ),
        ]
    )
