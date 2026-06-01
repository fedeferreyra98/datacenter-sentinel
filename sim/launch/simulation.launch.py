"""
DataCenter Sentinel — simulation launch file
Starts: Gazebo + robot_state_publisher + Nav2 + sentinel nodes

Usage:
    ros2 launch sentinel_sim simulation.launch.py
    ros2 launch sentinel_sim simulation.launch.py headless:=true  # Colab / CI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sentinel_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    # ── Launch arguments ──────────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='true',
        description='Run Gazebo without GUI (for Colab/CI)'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock'
    )
    csv_output_arg = DeclareLaunchArgument(
        'csv_output', default_value='/tmp/patrol_log.csv',
        description='Output path for patrol_log.csv'
    )

    headless      = LaunchConfiguration('headless')
    use_sim_time  = LaunchConfiguration('use_sim_time')
    csv_output    = LaunchConfiguration('csv_output')

    # ── File paths ────────────────────────────────────────────────────────
    world_file    = os.path.join(pkg_share, 'worlds', 'datacenter.world')
    urdf_file     = os.path.join(pkg_share, 'urdf',   'sentinel_robot.urdf')
    nav2_cfg_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── Gazebo (headless mode for Colab) ──────────────────────────────────
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen',
    )

    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=UnlessCondition(headless),
    )

    # ── Spawn robot ───────────────────────────────────────────────────────
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'sentinel_robot',
            '-file',   urdf_file,
            '-x', '1.5', '-y', '5.0', '-z', '0.1', '-Y', '0.0',
        ],
        output='screen',
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ── Static transform: map → odom (identity — no AMCL in iter 1) ──────
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── Nav2 bringup ──────────────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time':  use_sim_time,
            'params_file':   nav2_cfg_file,
            'use_lifecycle_mgr': 'true',
            'autostart':     'true',
        }.items(),
    )

    # ── Sentinel nodes (delayed to let Gazebo + Nav2 stabilise) ──────────
    virtual_sensor_node = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='sentinel_sim',
                executable='virtual_sensor_node',
                name='virtual_sensor_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'layout_file': os.path.join(
                        get_package_share_directory('sentinel_sim'),
                        '..', '..', '..', '..', 'src', 'datacenter-sentinel',
                        'config', 'datacenter_layout.json'
                    ),
                }],
            )
        ],
    )

    sensor_logger_node = TimerAction(
        period=35.0,
        actions=[
            Node(
                package='sentinel_sim',
                executable='sensor_logger_node',
                name='sensor_logger_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'output_file': csv_output,
                }],
            )
        ],
    )

    patrol_node = TimerAction(
        period=40.0,   # start after Nav2 is fully up and bonded
        actions=[
            Node(
                package='sentinel_sim',
                executable='patrol_node',
                name='patrol_node',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'waypoints_file': os.path.join(pkg_share, 'config', 'waypoints.yaml'),
                    'loop_patrol': False,  # single pass for iter 1
                }],
            )
        ],
    )

    return LaunchDescription([
        headless_arg,
        use_sim_time_arg,
        csv_output_arg,
        gazebo_server,
        gazebo_client,
        rsp,
        map_to_odom_tf,
        TimerAction(period=5.0, actions=[spawn_robot]),
        TimerAction(period=15.0, actions=[nav2_launch]),
        virtual_sensor_node,
        sensor_logger_node,
        patrol_node,
    ])
