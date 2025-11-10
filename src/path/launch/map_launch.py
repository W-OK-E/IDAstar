import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions


def generate_launch_description():
    ld = LaunchDescription()

    # === Map Server ===
    map_file_path = os.path.join(
        get_package_share_directory('path'),
        'maps',
        'map.yaml'
    )

    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}]
    )

    # === Lifecycle Manager ===
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    # === Spawn TurtleBot3 ===
    # Assuming you are using TurtleBot3 and Gazebo
    tb3_model = 'burger'  # or 'waffle', 'waffle_pi'
    tb3_pkg = get_package_share_directory('turtlebot3_bringup')
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_pkg, 'launch', 'rviz2.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # === Nav2 Bringup ===
    # nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'map': map_file_path,
    #         'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
    #     }.items()
    # )

    # === Add actions ===
    ld.add_action(map_server_cmd)
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(spawn_tb3)
    # ld.add_action(nav2_launch)

    return ld
