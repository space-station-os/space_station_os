from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch.event_handlers import OnProcessStart
import lifecycle_msgs.msg


def generate_launch_description():
    # Launch arguments
    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate_hz', default_value='10.0',
        description='Simulation step rate in Hz')
    duration_arg = DeclareLaunchArgument(
        'sim_duration_s', default_value='300.0',
        description='Simulation duration in seconds')

    sim_controller = LifecycleNode(
        package='ssos_sim',
        executable='simulation_controller',
        name='simulation_controller',
        namespace='',
        output='screen',
        parameters=[{
            'sim_rate_hz': LaunchConfiguration('sim_rate_hz'),
            'sim_duration_s': LaunchConfiguration('sim_duration_s'),
            'ic.altitude_km': 408.0,
            'ic.orbital_velocity_mps': 7660.0,
            'ic.atmospheric_o2_pct': 20.9,
            'ic.atmospheric_co2_ppm': 400.0,
            'ic.cabin_pressure_kpa': 101.3,
            'ic.cabin_temp_celsius': 22.0,
            'ic.solar_flux_w_m2': 1361.0,
            'ic.in_eclipse': False,
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
        sim_rate_arg,
        duration_arg,
        sim_controller,
        RegisterEventHandler(
            OnProcessStart(
                target_action=sim_controller,
                on_start=[
                    LogInfo(msg='simulation_controller started, configuring...'),
                    configure_event,
                ],
            )
        ),
        # Activate with: ros2 lifecycle set /simulation_controller activate
    ])