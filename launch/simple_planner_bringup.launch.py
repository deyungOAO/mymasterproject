#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
	gz_args = LaunchConfiguration('gz_args')
	map_yaml = LaunchConfiguration('map')
	use_sim_time = LaunchConfiguration('use_sim_time')

	# ---------- Launch arguments ----------
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

	declare_use_sim_time = DeclareLaunchArgument(
		'use_sim_time',
		default_value='true',
		description='Use simulation time'
	)

	# ---------- 1) Gazebo sim ----------
	# ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="..."
	gz_sim = IncludeLaunchDescription(
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

	# ---------- 2) Bridges (odom, scan, cmd_vel) ----------
	# ros2 run ros_gz_bridge parameter_bridge \
	#   /model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry \
	#   /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
	#   /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
	bridge = Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		name='gz_bridge_all',
		output='screen',
		arguments=[
			'/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
			'/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
			'/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
		]
	)

	# ---------- 3) Map server ----------
	# ros2 run nav2_map_server map_server --ros-args \
	#   -p yaml_filename:=map.yaml -p frame_id:=map
	map_server = Node(
		package='nav2_map_server',
		executable='map_server',
		name='map_server',
		output='screen',
		parameters=[{
			'use_sim_time': use_sim_time,
			'yaml_filename': map_yaml,
			'frame_id': 'map'
		}]
	)

	# ---------- 4) Lifecycle bringup for map_server ----------
	# ros2 run nav2_util lifecycle_bringup map_server
	map_lifecycle = Node(
		package='nav2_util',
		executable='lifecycle_bringup',
		name='map_server_lifecycle_bringup',
		output='screen',
		arguments=['map_server']
	)

	# ---------- 5) Static TF: map -> odom ----------
	# ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
	static_map_to_odom = Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		name='static_map_to_odom',
		arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
		output='screen'
	)

	# ---------- 6) Robot pose node ----------
	# ros2 run mymasterproject robot_pose_node.py \
	#   --ros-args -p odom_topic:=/model/vehicle_blue/odometry \
	#              -p robot_pose_topic:=/robot_pose
	robot_pose_node = Node(
		package='mymasterproject',
		executable='robot_pose_node.py',
		name='robot_pose_node',
		output='screen',
		parameters=[{
			'odom_topic': '/model/vehicle_blue/odometry',
			'robot_pose_topic': '/robot_pose',
			'use_sim_time': use_sim_time,
		}]
	)

	# ---------- 7) Planner node ----------
	# ros2 run mymasterproject planner_node.py --ros-args \
	#   -p map_topic:=/map \
	#   -p robot_pose_topic:=/robot_pose \
	#   -p goal_topic:=/goal_pose \
	#   -p path_topic:=/planned_path
	planner_node = Node(
		package='mymasterproject',
		executable='planner_node.py',
		name='planner_node',
		output='screen',
		parameters=[{
			'map_topic': '/map',
			'robot_pose_topic': '/robot_pose',
			'goal_topic': '/goal_pose',
			'path_topic': '/planned_path',
			'use_sim_time': use_sim_time,
		}]
	)

	# ---------- 8) Controller node ----------
	# ros2 run mymasterproject controller_node.py --ros-args \
	#   -p robot_pose_topic:=/robot_pose \
	#   -p path_topic:=/planned_path \
	#   -p cmd_vel_topic:=/cmd_vel
	controller_node = Node(
		package='mymasterproject',
		executable='controller_node.py',
		name='controller_node',
		output='screen',
		parameters=[{
			'robot_pose_topic': '/robot_pose',
			'path_topic': '/planned_path',
			'cmd_vel_topic': '/cmd_vel',
			'use_sim_time': use_sim_time,
		}]
	)

	return LaunchDescription([
		declare_gz_args,
		declare_map,
		declare_use_sim_time,

		gz_sim,
		bridge,
		map_server,
		map_lifecycle,
		static_map_to_odom,
		robot_pose_node,
		planner_node,
		controller_node,
	])
