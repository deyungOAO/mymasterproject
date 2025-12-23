#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	# =========================
	# Launch arguments
	# =========================
	gz_args = LaunchConfiguration('gz_args')
	map_yaml = LaunchConfiguration('map')
	params_file = LaunchConfiguration('params_file')
	use_sim_time = LaunchConfiguration('use_sim_time')
	use_collision_monitor = LaunchConfiguration('use_collision_monitor')

	declare_gz_args = DeclareLaunchArgument(
		'gz_args',
		default_value='-r /home/deyung/ros2_newws/src/mymasterproject/src/world/campus_world.sdf',
		description='Arguments for gz_sim (world, etc.)'
	)

	declare_map = DeclareLaunchArgument(
		'map',
		default_value='/home/deyung/ros2_newws/src/mymasterproject/maps/map.yaml',
		description='Full path to map YAML file'
	)

	declare_params_file = DeclareLaunchArgument(
		'params_file',
		default_value='/home/deyung/ros2_newws/src/mymasterproject/config/nav2_params.yaml',
		description='Full path to Nav2 params file'
	)

	declare_use_sim_time = DeclareLaunchArgument(
		'use_sim_time',
		default_value='true',
		description='Use simulation time'
	)

	declare_use_collision_monitor = DeclareLaunchArgument(
		'use_collision_monitor',
		default_value='false',
		description='Enable collision monitor'
	)

	# =========================
	# 1) Gazebo (ros_gz_sim)
	# ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="..."
	# =========================
	gz_sim_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution([
				FindPackageShare('ros_gz_sim'),
				'launch',
				'gz_sim.launch.py'
			])
		),
		launch_arguments={
			'gz_args': gz_args
		}.items()
	)

	# =========================
	# 2) static TF: map -> odom
	# ros2 run tf2_ros static_transform_publisher -18 -2 0 0 0 0 map odom
	# =========================
	static_map_to_odom = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_map_to_odom',
		arguments=['-18', '-2', '0', '0', '0', '0', 'map', 'odom'],
		output='screen'
	)

	# =========================
	# 3) odom TF broadcaster node (your package)
	# ros2 run mymasterproject odom_tf_broadcaster.py
	# (adjust executable name if needed)
	# =========================
	odom_tf_broadcaster = Node(
		package='mymasterproject',
		executable='odom_tf_broadcaster.py',  # if your entry point is different, change this
		name='odom_tf_broadcaster',
		output='screen'
	)

	# =========================
	# 4) Nav2 bringup
	# ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true ...
	# =========================
	nav2_bringup = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			PathJoinSubstitution([
				FindPackageShare('nav2_bringup'),
				'launch',
				'bringup_launch.py'
			])
		),
		launch_arguments={
			'use_sim_time': use_sim_time,
			'use_collision_monitor': use_collision_monitor,
			'map': map_yaml,
			'params_file': params_file
		}.items()
	)

	# =========================
	# 5) ros_gz_bridge for /odometry and /scan
	# ros2 run ros_gz_bridge parameter_bridge \
	#   /model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
	#   /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
	# =========================
	bridge_odom_scan = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		name='gz_bridge_odom_scan',
		arguments=[
			'/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
			'/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
		],
		output='screen'
	)

	# =========================
	# 6) static TF: base_link -> vehicle_blue/chassis/gpu_lidar
	# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link vehicle_blue/chassis/gpu_lidar
	# =========================
	static_base_to_lidar = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_base_to_lidar',
		arguments=['2', '0', '1', '0', '0', '0', 'base_link', 'vehicle_blue/chassis/gpu_lidar'],
		output='screen'
	)

	# =========================
	# 7) static TF: world -> map
	# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map
	# =========================
	static_world_to_map = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_world_to_map',
		arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
		output='screen'
	)

	# =========================
	# 8) RViz2
	# rviz2  (optionally you can specify a config file)
	# =========================
	#rviz2_node = Node(
	#	package='rviz2',
	#	executable='rviz2',
	#	name='rviz2',
	#	output='screen',
	#	parameters=[{'use_sim_time': use_sim_time}]
	#	# If you have a saved config:
	#	# arguments=['-d', '/path/to/your/config.rviz']
	#)

	# =========================
	# Launch description
	# =========================
	return LaunchDescription([
		declare_gz_args,
		declare_map,
		declare_params_file,
		declare_use_sim_time,
		declare_use_collision_monitor,

		gz_sim_launch,
		static_map_to_odom,
		odom_tf_broadcaster,
		nav2_bringup,
		bridge_odom_scan,
		static_base_to_lidar,
		static_world_to_map,
	])
