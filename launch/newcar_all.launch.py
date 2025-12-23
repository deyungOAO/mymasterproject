#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---- Launch arguments ----
    world_sdf = LaunchConfiguration('world_sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # ros_gz_sim launch file
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim_launch = f"{ros_gz_sim_share}/launch/gz_sim.launch.py"

    # Default world path (your current one)
    default_world = "/home/deyung/ros2_newws/src/mymasterproject/src/world/campus_world.sdf"

    # ---- Gazebo ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            # -r = run, start playing immediately
            'gz_args': ['-r ', world_sdf]
        }.items()
    )

    # ---- Bridges ----
    # NOTE: Keep these nodes alive; if you close them, ROS topics lose publishers.
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

    # Strongly recommended: /clock bridge so RViz + TF work in sim time
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

    # ---- TF from odometry (newcar/odom -> newcar/chassis) ----
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

    # ---- Static TF: chassis -> lidar frame (your exact command) ----
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_newcar_lidar',
        output='screen',
        # x y z roll pitch yaw parent child
        arguments=[
            '0.8', '0', '0.5',
            '0', '0', '0',
            'newcar/chassis', 'newcar/lidar_link/lidar'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_sdf',
            default_value=default_world,
            description='Absolute path to the Gazebo SDF world file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        gazebo,

        bridge_clock,
        bridge_cmd_vel,
        bridge_odom,
        bridge_scan,

        odom_tf,
        static_tf_lidar,
    ])
