#!/usr/bin/env python3
"""
Complete launch file for the autonomous navigation system.
Starts Gazebo, bridges, map server, planner, and controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='/home/deyung/ros2_newws/src/mymasterproject/src/world/campus_world.sdf',
        description='Full path to the Gazebo world file'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='map.yaml',
        description='Path to map yaml file'
    )
    
    planner_type_arg = DeclareLaunchArgument(
        'planner_type',
        default_value='fast_astar',
        description='Planner type: astar, fast_astar, or prm'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get launch configurations
    world_file = LaunchConfiguration('world_file')
    map_yaml = LaunchConfiguration('map_yaml')
    planner_type = LaunchConfiguration('planner_type')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 1. Launch Gazebo Simulator
    gazebo_simulator = ExecuteProcess(
        cmd=['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
             ['gz_args:=-r ', world_file]],
        output='screen',
        shell=True
    )

    # 2. Gazebo-ROS Bridges
    
    # Bridge for odometry
    bridge_odometry = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odometry',
        arguments=[
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge for laser scan
    bridge_laser = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_laser',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Bridge for cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Map Server (Lifecycle Node)
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_yaml},
            {'frame_id': 'map'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # 4. Map Server Lifecycle Bringup
    map_server_bringup = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        name='map_server_bringup',
        output='screen',
        arguments=['map_server']
    )

    # 5. Static Transform: map -> odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 6. Robot Pose Node (converts odometry to robot_pose)
    robot_pose_node = Node(
        package='mymasterproject',
        executable='robot_pose_node.py',
        name='robot_pose_node',
        output='screen',
        parameters=[
            {'odom_topic': '/model/vehicle_blue/odometry'},
            {'robot_pose_topic': '/robot_pose'},
            {'use_sim_time': use_sim_time}
        ]
    )

    # 7. Inflation Costmap Node
    inflation_costmap_node = Node(
        package='mymasterproject',
        executable='simple_inflation_costmap.py',
        name='inflation_costmap_node',
        output='screen',
        parameters=[
            {'input_map_topic': '/map'},
            {'output_costmap_topic': '/global_costmap/costmap'},
            {'inflation_radius': 1.4},  # Robot size
            {'use_sim_time': use_sim_time}
        ]
    )

    # 8. Planner Node (choose based on planner_type)
    # Note: We'll use a single node and rely on the planner_type parameter
    # You can modify this to conditionally launch different planners if needed
    
    # Fast A* Planner (default)
    planner_node = Node(
        package='mymasterproject',
        executable='fast_a_star_planner.py',  # Change to 'prm_planner.py' or 'a_star_planner_improved.py' as needed
        name='planner_node',
        output='screen',
        parameters=[
            {'map_topic': '/map'},
            {'costmap_topic': '/global_costmap/costmap'},
            {'robot_pose_topic': '/robot_pose'},
            {'goal_topic': '/goal_pose'},
            {'path_topic': '/planned_path'},
            {'use_sim_time': use_sim_time},
            # Fast A* parameters
            {'use_bidirectional': True},
            {'heuristic_weight': 1.2},
            {'skip_proximity_check': False},
            {'early_termination_radius': 5},
            {'inflation_radius_m': 1.4},
            {'safety_distance_m': 0.3},
            {'cost_weight': 3.0},
            {'occupied_threshold': 50},
        ]
    )

    # 9. Controller Node
    controller_node = Node(
        package='mymasterproject',
        executable='controller_node.py',
        name='controller_node',
        output='screen',
        parameters=[
            {'robot_pose_topic': '/robot_pose'},
            {'path_topic': '/planned_path'},
            {'cmd_vel_topic': '/cmd_vel'},
            {'use_sim_time': use_sim_time},
            # Controller parameters
            {'lookahead_distance': 1.5},
            {'max_linear_velocity': 2.0},
            {'max_angular_velocity': 1.5},
            {'goal_tolerance': 0.2},
        ]
    )

    # Create launch description with timed delays for proper startup
    return LaunchDescription([
        # Launch arguments
        world_file_arg,
        map_yaml_arg,
        planner_type_arg,
        use_sim_time_arg,
        
        # 1. Start Gazebo first
        gazebo_simulator,
        
        # 2. Wait 3 seconds, then start bridges
        TimerAction(
            period=3.0,
            actions=[
                bridge_odometry,
                bridge_laser,
                bridge_cmd_vel,
            ]
        ),
        
        # 3. Wait 5 seconds, then start static TF and map server
        TimerAction(
            period=5.0,
            actions=[
                static_tf_map_odom,
                map_server,
            ]
        ),
        
        # 4. Wait 7 seconds, then bring up map server lifecycle
        TimerAction(
            period=7.0,
            actions=[
                map_server_bringup,
            ]
        ),
        
        # 5. Wait 9 seconds, then start robot pose and costmap nodes
        TimerAction(
            period=9.0,
            actions=[
                robot_pose_node,
                inflation_costmap_node,
            ]
        ),
        
        # 6. Wait 11 seconds, then start planner
        TimerAction(
            period=11.0,
            actions=[
                planner_node,
            ]
        ),
        
        # 7. Wait 13 seconds, then start controller
        TimerAction(
            period=13.0,
            actions=[
                controller_node,
            ]
        ),
    ])