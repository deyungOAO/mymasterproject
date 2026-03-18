# bringup_nav2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    params_file  = LaunchConfiguration('params_file')
    map_yaml     = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value='/home/deyung/ros2_newws/src/mymasterproject/nav2_params.yaml'),
        DeclareLaunchArgument('map', default_value='/home/deyung/ros2_newws/src/mymasterproject/maps/map.yaml'),

        # Map server
        Node(
            package='nav2_map_server', executable='map_server', name='map_server', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
        ),
        # AMCL
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[params_file]
        ),

        # Planner, Controller, Recoveries, BT Navigator (all lifecycle nodes)
        Node(package='nav2_planner',    executable='planner_server',    name='planner_server',    output='screen', parameters=[params_file]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors',  executable='behavior_server',   name='behavior_server',   output='screen', parameters=[params_file]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',    name='bt_navigator',      output='screen', parameters=[params_file]),

        # Lifecycle manager to activate everything
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator'
                ]
            }]
        ),
    ])
