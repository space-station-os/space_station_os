import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, TimerAction
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
import lifecycle_msgs.msg


def _configure(node):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))


def _activate(node):
    return EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))


def generate_launch_description():
    share = get_package_share_directory('ssos_eps')

    delay_arg = DeclareLaunchArgument(
        'activation_delay', default_value='10.0',
        description='Seconds before all nodes are activated')
    configure_arg = DeclareLaunchArgument(
        'configure_delay', default_value='3.0',
        description='Seconds before all nodes are configured')

    def config(name):
        return os.path.join(share, 'config', name)

    system_manager = LifecycleNode(
        package='ssos_core', executable='system_manager', name='system_manager',
        namespace='', output='screen')
    sim_controller = LifecycleNode(
        package='ssos_sim', executable='simulation_controller',
        name='simulation_controller', namespace='', output='screen',
        parameters=[{'sim_duration_s': 1.0e9}])
    eps = LifecycleNode(
        package='ssos_eps', executable='eps_node', name='eps_node',
        namespace='', output='screen',
        parameters=[config('eps_parameters.yaml'),
                    config('solar_parameters.yaml'),
                    config('battery_parameters.yaml'),
                    config('bcdu_parameters.yaml')])

    all_nodes = [system_manager, sim_controller, eps]

    configure_all = TimerAction(
        period=LaunchConfiguration('configure_delay'),
        actions=[LogInfo(msg='Configuring all nodes...')] +
                [_configure(n) for n in all_nodes])

    activate_all = TimerAction(
        period=LaunchConfiguration('activation_delay'),
        actions=[LogInfo(msg='Activating all nodes...')] +
                [_activate(n) for n in all_nodes])

    return LaunchDescription(
        [delay_arg, configure_arg] + all_nodes + [configure_all, activate_all])
