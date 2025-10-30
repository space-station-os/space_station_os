from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ------------------------------------------------------------------
    # Paths
    # ------------------------------------------------------------------
    pkg_share = get_package_share_directory('space_station_gnc')

    # NOTE: Adjust this if your model is a .xacro or has a different name.
    # For xacro: replace Command(['cat ', urdf_path]) with Command(['xacro ', urdf_path])
    urdf_path = PathJoinSubstitution([
        FindPackageShare('space_station_gnc'),
        'urdf',
        'space_station_eagle.xacro'
    ])

    # ------------------------------------------------------------------
    # Arguments
    # ------------------------------------------------------------------
    table_mode_arg = DeclareLaunchArgument(
        'table_mode',
        default_value='sixdof_phys',
        description='Thruster table mode to use inside ThrusterMatrix'
    )

    # ------------------------------------------------------------------
    # Parameters (GNC / controller)
    # ------------------------------------------------------------------
    thruster_params = {
        # PD gains
        'kp_thruster': 1_000_000.0,
        'kd_thruster': 1_000_000.0,

        # LPF for feedback signals (control_torque.cpp)
        'thruster_omega_lpf_tau': 1.0,
        'thruster_angle_lpf_tau': 0.0,
        'lpf_dt': 0.1,

        # Thruster tables & properties
        'thruster_tables_yaml':     'package://space_station_gnc/config/eagle_thruster_table.yaml',
        'thruster_properties_yaml': 'package://space_station_gnc/config/eagle_thruster_properties.yaml',
        'table_mode': LaunchConfiguration('table_mode'),
    }

    # ------------------------------------------------------------------
    # Parameters (plant / physics)
    # ------------------------------------------------------------------
    phys_params = {
        # Loop periods
        'timing.torque_dt': 0.1,       # dynamics step
        'timing.pub_dt':    0.1,       # publish step
        'timing.publish_every': 10,    # reduce logging frequency

        # Inertia and orbit
        'dynamics.J.xx': 4.2e6,
        'dynamics.J.yy': 4.2e6,
        'dynamics.J.zz': 6.7e6,
        'dynamics.mu':   3.986e14,
        'dynamics.r_orbit': 7.0e6,

        # Optional initial states (uncomment if needed)
        # 'initial.attitude': [0.0, 0.0, 0.0, 1.0],
        # 'initial.angvel':  [0.0, 0.0, 0.0],
        # 'initial.angacc':  [0.0, 0.0, 0.0],
    }

    # ------------------------------------------------------------------
    # Nodes
    # ------------------------------------------------------------------
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            # If your model is a .xacro, use: Command(['xacro ', urdf_path])
            'robot_description': ParameterValue(
                Command(['xacro', ' ', urdf_path]),  #
                value_type=str
            ),
        }],

        output='screen'
    )

    #torque_collector = Node(
    #    package='space_station_gnc',
    #    executable='torque_collector',
    #    name='torque_collector',
    #    arguments=['--ros-args',
    #            '--log-level', 'warn',
    #            '--log-level', 'torque_collector:=warn'],
    #    output='screen'
    #)

    control_torque = Node(
        package='space_station_gnc',
        executable='control_torque',
        name='control_torque',
        parameters=[thruster_params],
        arguments=['--ros-args',
                '--log-level', 'info',
                '--log-level', 'control_torque:=info'],
        output='screen'
    )

    physics_motion = Node(
        package='space_station_gnc',
        executable='physics_motion',
        name='physics_motion',
        parameters=[phys_params],
        arguments=['--ros-args',
               '--log-level', 'warn',
               '--log-level', 'physics_motion:=warn'],
        output='screen'
    )

    physics_sensor = Node(
        package='space_station_gnc',
        executable='physics_sensor',
        name='physics_sensor',
        arguments=['--ros-args',
               '--log-level', 'warn',
               '--log-level', 'physics_sensor:=warn'],
        output='screen'
    )

    sense_estimate = Node(
        package='space_station_gnc',
        executable='sense_estimate',
        name='sense_estimate',
        arguments=['--ros-args',
                '--log-level', 'warn',
                '--log-level', 'sense_estimate:=warn'],
        output='screen'
    )

    return LaunchDescription([
        table_mode_arg,
        robot_state_pub,
        #torque_collector,
        control_torque,
        physics_motion,
        physics_sensor,
        sense_estimate,
    ])

