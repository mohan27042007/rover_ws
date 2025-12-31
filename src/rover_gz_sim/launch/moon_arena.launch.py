import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    rover_gz_sim_pkg = get_package_share_directory('rover_gz_sim')
    rover_worlds_pkg = get_package_share_directory('rover_worlds')

    # Path to the moon arena model SDF
    moon_arena_sdf_path = os.path.join(rover_worlds_pkg, 'models', 'my_dae_model', 'model.sdf')

    # Path to models directory to add to GZ_SIM_RESOURCE_PATH
    models_path = os.path.join(rover_worlds_pkg, 'models')

    # Append to GZ_SIM_RESOURCE_PATH so Gazebo can find the models
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # Include the main robot launch file
    # We use 'empty.sdf' as the base world
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_gz_sim_pkg, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'world': 'empty.sdf',
            'x': '0.0',
            'y': '0.0',
            'z': '1.3', # Start high above the arena
        }.items()
    )

    # Node to spawn the moon arena
    spawn_arena_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'moon_arena',
            '-file', moon_arena_sdf_path,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    return LaunchDescription([
        set_env_vars,
        robot_launch,
        spawn_arena_node
    ])
