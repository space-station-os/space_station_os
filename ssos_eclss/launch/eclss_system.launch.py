
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
    share = get_package_share_directory('ssos_eclss')

    delay_arg = DeclareLaunchArgument(
        'activation_delay', default_value='20.0',
        description='Seconds before all nodes are activated')
    configure_arg = DeclareLaunchArgument(
        'configure_delay', default_value='3.0',
        description='Seconds before all nodes are configured')

    def eclss_node(executable, name, config_file):
        return LifecycleNode(
            package='ssos_eclss', executable=executable, name=name,
            namespace='', output='screen',
            parameters=[os.path.join(share, 'config', config_file)])

    system_manager = LifecycleNode(
        package='ssos_core', executable='system_manager', name='system_manager',
        namespace='', output='screen')
    sim_controller = LifecycleNode(
        package='ssos_sim', executable='simulation_controller',
        name='simulation_controller', namespace='', output='screen')

    ars = eclss_node('ars_node', 'ars_node', 'ars_parameters.yaml')
    ogs = eclss_node('ogs_node', 'ogs_node', 'ogs_parameters.yaml')
    wrs = eclss_node('wrs_node', 'wrs_node', 'wrs_parameters.yaml')
    cabin = eclss_node('cabin_node', 'cabin_node', 'cabin_parameters.yaml')

    all_nodes = [system_manager, sim_controller, ars, ogs, wrs, cabin]

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
