"""SSOS mission-control launch.

CURRENT TEST CONFIG: ECLSS only, with ssos_core (system_manager) and ssos_sim
(simulation_controller). Sequencing:
  - all processes start at t=0,
  - system_manager + simulation_controller are configured on start and ACTIVATED
    at t=10s,
  - the ECLSS nodes self-activate at t=11s (autostart_delay_ms) -- i.e. after
    core+sim are active, so registration with the system_manager succeeds and
    the simulator's /sim/world_state is already flowing.

Thermal / EPS / GNC / orbit are commented out for this ECLSS-focused test;
re-enable the blocks at the bottom to bring the full station back.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (EmitEvent, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
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
    space_station_pkg = get_package_share_directory('space_station')

    # ---- Mission-control GUI ----
    gui = Node(
        package='space_station',
        executable='space_station',
        name='space_station_gui_node',
        output='screen',
    )

    # ---- ssos_core: system_manager (lifecycle) ----
    system_manager = LifecycleNode(
        package='ssos_core', executable='system_manager', name='system_manager',
        namespace='', output='screen')

    # ---- ssos_sim: simulation_controller (lifecycle) ----
    sim_controller = LifecycleNode(
        package='ssos_sim', executable='simulation_controller',
        name='simulation_controller', namespace='', output='screen')

    # Configure core + sim as soon as their processes start.
    configure_core_sim = [
        RegisterEventHandler(OnProcessStart(
            target_action=system_manager,
            on_start=[LogInfo(msg='system_manager started; configuring...'),
                      _configure(system_manager)])),
        RegisterEventHandler(OnProcessStart(
            target_action=sim_controller,
            on_start=[LogInfo(msg='simulation_controller started; configuring...'),
                      _configure(sim_controller)])),
    ]

    # Activate core + sim at t=10s.
    activate_core_sim = TimerAction(
        period=10.0,
        actions=[LogInfo(msg='Activating system_manager + simulation_controller...'),
                 _activate(system_manager), _activate(sim_controller)])

    # ---- ECLSS (self-activates at t=11s, after core+sim) ----
    eclss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(space_station_pkg, 'launch', 'eclss.launch.py')
        ),
        launch_arguments={'autostart_delay_ms': '11000'}.items(),
    )

    ld = LaunchDescription([
        gui,
        system_manager,
        sim_controller,
        activate_core_sim,
        eclss,
    ])
    for handler in configure_core_sim:
        ld.add_action(handler)
    return ld

    # =====================================================================
    # FULL STATION (re-enable for the complete system) -- commented out for
    # the ECLSS-only test above.
    #
    #   IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #       os.path.join(space_station_pkg, 'launch', 'thermals.launch.py'))),
    #   IncludeLaunchDescription(PythonLaunchDescriptionSource(
    #       os.path.join(space_station_pkg, 'launch', 'eps.launch.py'))),
    #   Node(package='space_station_eps', executable='solar_power',
    #        name='solar_power_node', output='screen'),
    #   IncludeLaunchDescription(
    #       PythonLaunchDescriptionSource(os.path.join(
    #           get_package_share_directory('space_station_gnc'),
    #           'launch', 'gnc_core.launch.py')),
    #       launch_arguments={'quiet_torque': 'true', 'quiet_control': 'true',
    #                         'quiet_motion': 'true', 'quiet_sensor': 'true',
    #                         'quiet_estimate': 'true'}.items()),
    #   Node(package='space_station_gnc', executable='orbit_dynamics',
    #        name='orbit_dynamics_node', output='log',
    #        parameters=[os.path.join(get_package_share_directory('space_station_gnc'),
    #                                 'config', 'orbit_dynamics.yaml')]),
    #   Node(package='space_station_gnc', executable='orbit_dynamics_mock',
    #        name='orbit_dynamics_mock', output='log'),
    # =====================================================================
