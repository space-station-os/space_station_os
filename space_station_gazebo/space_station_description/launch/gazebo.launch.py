import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_ssos = get_package_share_directory('space_station_description')

    # Set Gazebo model path
    gazebo_models_path = os.path.join(pkg_ssos)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + gazebo_models_path
    
    earth_model_path = os.path.join(pkg_ssos, 'models')
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.environ.get("GZ_SIM_RESOURCE_PATH", "") + os.pathsep + earth_model_path

    # Launch arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false', description='Open RViz')
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='urdf.rviz', description='RViz config file')
    world_arg = DeclareLaunchArgument('world', default_value=PathJoinSubstitution([pkg_ssos, 'worlds', 'orbit.sdf']), description='Path to Gazebo world')
    model_arg = DeclareLaunchArgument('model', default_value='space_station.xacro', description='URDF filename')
    sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    controller_config_arg = DeclareLaunchArgument('controller', default_value=PathJoinSubstitution([pkg_ssos, 'config', 'controller.yaml']), description='Controller YAML path')

    urdf_file_path = PathJoinSubstitution([pkg_ssos, "urdf", LaunchConfiguration('model')])

    # Launch world
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ssos, 'launch', 'world.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(Command(['xacro', ' ', urdf_file_path]), value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )

    # Spawn robot in Gazebo
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "spacestation", "-topic", "robot_description", "-x", "0.0", "-y", "0.0", "-z", "0.085"],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Bridge
    gz_bridge_node =Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ssos, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    imu_state=Node(
            package="controller_manager",
            executable="spawner",
            arguments=["imu_broadcaster"],
        )
    # Optional RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_ssos, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ros2_control manager
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            LaunchConfiguration('controller'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Position controller spawner
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['solar_controller'],
        output='screen'
    )
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    
    

    return LaunchDescription([
        rviz_arg,
        rviz_config_arg,
        world_arg,
        model_arg,
        sim_time_arg,
        controller_config_arg,
        world_launch,
        gz_bridge_node,
        rviz_node,
        robot_state_publisher_node,
        spawn_urdf_node,
        ros2_control_node,
        # imu_state,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_urdf_node,
                on_exit=[joint_state_broadcaster_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[position_controller_spawner]
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=position_controller_spawner,
        #         on_exit=[pose_commander_node]
        #     )
        # )
    ])