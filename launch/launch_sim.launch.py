import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = "autonomous_bot"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': "true"}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(pkg_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    world_file_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'textile_world.sdf')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={'world': world_file_path}.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            "-topic", "robot_description",
            "-entity", "autonomous_bot",
            "-x", "2.5",  # X position
            "-y", "0",  # Y position
            "-z", "1",  # Z position
            "-Y", "1.5",
            "-R", "0",
            "-P", "0",
        ],
        output="screen"
    )

    gazebo_arguments = [
        ("world", world_file_path),
        ("paused", "false"), 
        ("use_sim_time", "true"),
        ("extra_gazebo_args", "-s 2.5") 
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("gazebo_ros"), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=gazebo_arguments
    )

    return LaunchDescription([
        rsp, gazebo, spawn_entity, twist_mux
    ])
