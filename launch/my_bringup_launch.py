#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
	# --- Launch configurations (what you pass with ros2 launch ...) ---
	namespace = LaunchConfiguration('namespace')
	use_sim_time = LaunchConfiguration('use_sim_time')
	autostart = LaunchConfiguration('autostart')
	params_file = LaunchConfiguration('params_file')
	map_yaml_file = LaunchConfiguration('map')

	# --- Package paths ---
	nav2_bringup_dir = get_package_share_directory('nav2_bringup')
	nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')

	my_pkg_dir = get_package_share_directory('mymasterproject')
	my_launch_dir = os.path.join(my_pkg_dir, 'launch')

	# --- Localization (map_server + AMCL) from nav2_bringup ---
	localization_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(nav2_launch_dir, 'localization_launch.py')
		),
		launch_arguments={
			'namespace': namespace,
			'use_sim_time': use_sim_time,
			'map': map_yaml_file,
			'params_file': params_file,
		}.items(),
	)

	# --- Navigation stack (planner, controller, costmaps, bt_navigator, etc.) ---
	#     This uses YOUR custom my_navigation_launch.py (no docking_server).
	navigation_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(my_launch_dir, 'my_navigation_launch.py')
		),
		launch_arguments={
			'namespace': namespace,
			'use_sim_time': use_sim_time,
			'autostart': autostart,
			'params_file': params_file,
			'use_composition': 'False',
			'use_respawn': 'False',
			'container_name': 'nav2_container',
		}.items(),
	)

	return LaunchDescription([
		# --- Arguments you can override from the command line ---
		DeclareLaunchArgument(
			'namespace',
			default_value='',
			description='Top-level namespace'
		),
		DeclareLaunchArgument(
			'use_sim_time',
			default_value='true',
			description='Use /clock from simulation'
		),
		DeclareLaunchArgument(
			'autostart',
			default_value='true',
			description='Automatically start up the Nav2 stack'
		),
		DeclareLaunchArgument(
			'params_file',
			default_value=os.path.join(
				my_pkg_dir,
				'config',
				'nav2_params.yaml'
			),
			description='Full path to the ROS2 parameters file to use for Nav2'
		),
		DeclareLaunchArgument(
			'map',
			default_value=os.path.join(
				my_pkg_dir,
				'maps',
				'map.yaml'
			),
			description='Full path to map yaml file'
		),

		# --- Run localization + navigation together ---
		GroupAction([
			localization_launch,
			navigation_launch,
		]),
	])
