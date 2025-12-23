#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- Launch arguments ----
    world_sdf = LaunchConfiguration('world_sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ros_gz_sim launch file
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')

    # ✅ Portable default world: inside this package
    # Put campus_world.sdf in: mymasterproject/world/campus_world.sdf (recommended)
    default_world = PathJoinSubstitution([
        FindPackageShare('mymasterproject'),
        'world',
        'campus_world.sdf'
    ])

    # ---- Gazebo ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            # IMPORTANT: gz_args should be a single string
            'gz_args': [LaunchConfiguration('gz_args')]
        }.items()
    )

    # ---- Bridges ----
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        output='screen',
        arguments=[
            '/model/newcar/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        output='screen',
        arguments=[
            '/model/newcar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_scan',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ---- TF from odometry ----
    odom_tf = Node(
        package='mymasterproject',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_topic': '/model/newcar/odometry'},
        ],
    )

    # ---- Static TF: chassis -> lidar (new-style args, no warning) ----
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_newcar_lidar',
        output='screen',
        arguments=[
            '--x', '0.8', '--y', '0', '--z', '0.5',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'newcar/chassis',
            '--child-frame-id', 'newcar/lidar_link/lidar',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        # Optional: reduce FastDDS SHM spam on some machines
        SetEnvironmentVariable('FASTDDS_SHM_TRANSPORT', '0'),

        DeclareLaunchArgument(
            'world_sdf',
            default_value=default_world,
            description='Path to the Gazebo SDF world file (defaults to package world/campus_world.sdf)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            # This becomes the actual gz_args passed to ros_gz_sim
            'gz_args',
            default_value=['-r ', world_sdf],
            description='Arguments passed to Gazebo (default: -r <world_sdf>)'
        ),

        gazebo,

        bridge_clock,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,

        odom_tf,
        static_tf_lidar,
    ])
