from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

print("✅ RUNNING newcar_launch.py (CLEAN: odom-only, no ground truth)")


def generate_launch_description():
    plugin_dir = "/home/deyung/ros2_newws/install/actor_pose_ros2/lib"

    # ==================== ENV ====================
    set_gz_sim_system_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=f"{plugin_dir}:$GZ_SIM_SYSTEM_PLUGIN_PATH"
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_PLUGIN_PATH",
        value=f"{plugin_dir}:$GZ_PLUGIN_PATH"
    )

    # ==================== CLOCK ====================
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ==================== URDF ====================
    urdf_file = '/home/deyung/ros2_newws/src/mymasterproject/src/newcar/newcar.urdf'
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
            "frame_prefix": "ackermann_car/"
        }],
    )

    # ==================== GAZEBO ====================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': '-r /home/deyung/ros2_newws/src/mymasterproject/src/world/campus_world.sdf'
        }.items()
    )

    # ==================== SPAWN ====================
    spawn_entity = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'ackermann_car',
                '-x', '-20', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        )]
    )

    # ==================== BRIDGES ====================
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Single odom bridge → /odom (used by everything)
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/ackermann_car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[('/model/ackermann_car/odometry', '/odom')]
    )

    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ==================== TF ====================
    # Static: map -> odom (spawn offset)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['-20', '0', '0', '0', '0', '0', 'map', 'ackermann_car/odom'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Static: base_footprint -> lidar scan frame
    static_tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_basefootprint_to_scanframe',
        output='screen',
        arguments=[
            '--x', '0.4572', '--y', '0', '--z', '1',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'ackermann_car/base_footprint',
            '--child-frame-id', 'ackermann_car/base_footprint/lidar'
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Dynamic: odom -> base_footprint (from odometry)
    odom_to_tf = ExecuteProcess(
        cmd=['python3',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/odom_to_tf.py',
             '--ros-args',
             '-p', 'gz_odom_topic:=/carodom_world',
             '-p', 'spawn_x:=-20.0',
             '-p', 'spawn_y:=0.0',
             '-p', 'use_sim_time:=true',
             ],
        output='screen'
    )
    bridge_carodom = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/carodomfromGazebo@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
    output='screen',
    parameters=[{'use_sim_time': True}]
    )
    carodom_relay = ExecuteProcess(
    cmd=['python3',
         '/home/deyung/ros2_newws/src/mymasterproject/scripts/carodom_relay.py',
         '--ros-args',
         '-p', 'use_sim_time:=true',
         ],
    output='screen'
    )
    # ==================== ROBOT POSE ====================
    robot_pose_node = ExecuteProcess(
        cmd=['python3',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/robot_pose_node.py',
             '--ros-args',
             '-p', 'odom_topic:=/carodom_world',
             '-p', 'robot_pose_topic:=/robot_pose',
             '-p', 'use_sim_time:=true',
             ],
        output='screen'
    )

    # ==================== MAP SERVER ====================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': '/home/deyung/ros2_newws/src/mymasterproject/maps/map.yaml',
            'frame_id': 'map',
            'use_sim_time': True
        }]
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_lifecycle',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server'],
            'use_sim_time': True
        }]
    )

    # ==================== COSTMAPS ====================
    inflation_costmap = ExecuteProcess(
        cmd=['python3',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/simple_inflation_costmap.py',
             '--ros-args',
             '-p', 'input_map_topic:=/map',
             '-p', 'output_costmap_topic:=/global_costmap/costmap',
             '-p', 'inflation_radius:=2.0',
             '-p', 'robot_radius:=1.0',
             '-p', 'cost_scaling_factor:=2.0',
             '-p', 'use_sim_time:=true',
             ],
        output='screen'
    )

    local_costmap = ExecuteProcess(
        cmd=['python3',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/local_costmap_generator.py',
             '--ros-args',
             '-p', 'costmap_width:=20.0',
             '-p', 'costmap_height:=20.0',
             '-p', 'resolution:=0.1',
             '-p', 'robot_radius:=1.0',
             '-p', 'inflation_radius:=1.5',
             '-p', 'obstacle_threshold:=0.8',
             '-p', 'scan_topic:=/scan',
             '-p', 'odom_topic:=/carodom_world',
             '-p', 'costmap_topic:=/local_costmap',
             '-p', 'update_frequency:=5.0',
             '-p', 'use_sim_time:=true',
             ],
        output='screen'
    )

    # ==================== PLANNERS ====================
    astar_planner = ExecuteProcess(
        cmd=['python3', '-u',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/planner_node.py',
             '--ros-args',
             '-p', 'map_topic:=/map',
             '-p', 'costmap_topic:=/global_costmap/costmap',
             '-p', 'robot_pose_topic:=/robot_pose',
             '-p', 'goal_topic:=/goal_pose',
             '-p', 'path_topic:=/planned_path',
             '-p', 'use_bidirectional:=true',
             '-p', 'heuristic_weight:=1.2',
             '-p', 'max_iterations:=50000',
             '-p', 'inflation_radius_m:=1.4',
             '-p', 'cost_weight:=3.0',
             '-p', 'allow_diagonal:=true',
             '-p', 'enable_keyboard_control:=true',
             '-p', 'smooth_path:=true',
             '-p', 'use_sim_time:=true',
             ],
        output='screen',
        emulate_tty=True,
    )

    dwa_local_planner = ExecuteProcess(
        cmd=['python3',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/dwa_local_planner.py',
             '--ros-args',
             '-p', 'robot_pose_topic:=/robot_pose',
             '-p', 'path_topic:=/planned_path',
             '-p', 'cmd_vel_topic:=/cmd_vel',
             '-p', 'scan_topic:=/scan',
             '-p', 'odom_topic:=/carodom_world',
             '-p', 'local_path_topic:=/dwa_local_path',
             '-p', 'goal_topic:=/goal_pose',
             '-p', 'wp_skip_threshold:=80',
             '-p', 'wp_skip_max:=5',
             '-p', 'replan_after_s:=8.0',
             '-p', 'max_speed:=5.0',
             '-p', 'min_speed:=-0.3',
             '-p', 'max_yaw_rate:=0.8',
             '-p', 'max_accel:=1.0',
             '-p', 'max_dyaw_rate:=0.5',
             '-p', 'escape_creep_speed:=0.3',
             '-p', 'velocity_samples:=8',
             '-p', 'yaw_rate_samples:=15',
             '-p', 'predict_time:=2.0',
             '-p', 'dt:=0.1',
             '-p', 'heading_cost_gain:=2.0',
             '-p', 'distance_cost_gain:=1.0',
             '-p', 'velocity_cost_gain:=0.3',
             '-p', 'obstacle_cost_gain:=3.0',
             '-p', 'robot_radius:=1.5',
             '-p', 'goal_tolerance:=0.2',
             '-p', 'use_sim_time:=true',
             ],
        output='screen'
    )

    # ==================== RVIZ ====================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ==================== LAUNCH ====================
    return LaunchDescription([
        set_gz_sim_system_plugin_path,
        set_gz_plugin_path,

        bridge_clock,
        gazebo,
        robot_state_publisher,
        spawn_entity,

        # Bridges
        bridge_cmd_vel,
        bridge_odom,
        bridge_lidar,
        bridge_joint_states,

        # TF chain: map -> odom (static) -> base_footprint (dynamic)
        static_tf_map_odom,
        static_tf_scan,
        odom_to_tf,
        bridge_carodom,
        carodom_relay,
        # Pose
        robot_pose_node,

        # Nav stack
        TimerAction(period=7.0,  actions=[map_server, map_server_lifecycle]),
        TimerAction(period=9.0,  actions=[inflation_costmap]),
        TimerAction(period=10.0, actions=[local_costmap]),
        TimerAction(period=11.0, actions=[astar_planner]),
        TimerAction(period=12.0, actions=[ExecuteProcess(
            cmd=['python3',
             '/home/deyung/ros2_newws/src/mymasterproject/scripts/actor_safety_node.py',
             '--ros-args',
             '-p', 'use_sim_time:=true',
             '-p', 'stop_dist:=2.0',
             '-p', 'warn_dist:=4.0',
             '-p', 'clear_dist:=4.5',
             '-p', 'car_slow_dist:=4.0',
             '-p', 'car_crawl_dist:=1.5',
             '-p', 'actor_names:=actor_walking1,actor_walking2,actor_walking3,actor_walking4',
            ],
            output='screen'
        )]),
        TimerAction(period=13.0, actions=[dwa_local_planner]),
        TimerAction(period=15.0, actions=[rviz]),
        TimerAction(period=17.0, actions=[ExecuteProcess(
            cmd=['python3',
         '/home/deyung/ros2_newws/src/mymasterproject/scripts/nav_benchmark_multi.py',
         '--ros-args',
         '-p', 'use_sim_time:=true',
         '-p', 'goal_x:=16.0',
         '-p', 'goal_y:=0.0',
         '-p', 'num_runs:=10',        # ← change this for how many loops
         '-p', 'spawn_x:=-20.0',
         '-p', 'spawn_y:=0.0',
         '-p', 'model_name:=ackermann_car',
         '-p', 'reset_wait_s:=5.0',
         '-p', 'goal_publish_delay:=3.0',
        ],
        output='screen'
        )]),
    ])