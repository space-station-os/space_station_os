import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():
    pkg_space_station_description = get_package_share_directory('space_station_description')

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg_space_station_description, 'worlds', 'orbit.sdf'),
        description='Path to the Gazebo world file'
    )


    pkg_space_station_description = get_package_share_directory('space_station_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add your own gazebo library path here
    gazebo_models_path = "/home/siddarth/ssos_ws/src/space_station_gazebo/space_station_description/urdf"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_space_station_description,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        TextSubstitution(text=' -r -v  --physics-engine gz-physics-bullet-featherstone-plugin ')],
        'on_exit_shutdown': 'true'}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject