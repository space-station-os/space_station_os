from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
import lifecycle_msgs.msg


def generate_launch_description():
    system_manager = LifecycleNode(
        package='ssos_core',
        executable='system_manager',
        name='system_manager',
        namespace='',
        output='screen',
        parameters=[{
            'heartbeat_timeout_s': 5.0,
            'eval_period_s': 1.0,
        }],
    )

    # Auto-configure on start
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: True,
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    return LaunchDescription([
        system_manager,
        RegisterEventHandler(
            OnProcessStart(
                target_action=system_manager,
                on_start=[
                    LogInfo(msg='system_manager started, configuring...'),
                    configure_event,
                ],
            )
        ),
        # To activate after configure completes:
        #   ros2 lifecycle set /system_manager activate
    ])