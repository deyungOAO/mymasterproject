from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch arguments ---
    world_sdf = LaunchConfiguration('world_sdf')
    map_yaml = LaunchConfiguration('map_yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # If you want the lidar offset correct, set these
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    lidar_roll = LaunchConfiguration('lidar_roll')
    lidar_pitch = LaunchConfiguration('lidar_pitch')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_sdf',
            default_value='',
            description='Absolute path to the Gazebo world SDF. Leave empty to not start Gazebo from this launch.'
        ),
        DeclareLaunchArgument(
            'map_yaml',
            default_value='map.yaml',
            description='Path to map yaml (absolute or relative to current working dir).'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation time.'
        ),

        # LiDAR pose relative to newcar/chassis -> newcar/lidar_link/lidar
        DeclareLaunchArgument('lidar_x', default_value='0.8'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.5'),
        DeclareLaunchArgument('lidar_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),

        # --- (Optional) Start Gazebo Harmonic ---
        # If you already start Gazebo elsewhere, set world_sdf:=""
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py',
                'gz_args:=', ['-r ', world_sdf]
            ],
            output='screen',
            condition=None
        ),

        # --- Bridges (adjust if you already bridge elsewhere) ---
        # /scan
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_scan',
            output='screen',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # /model/newcar/odometry -> nav_msgs/Odometry
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_odom',
            output='screen',
            arguments=[
                '/model/newcar/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # /cmd_vel (if you want command control wired)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge_cmd_vel',
            output='screen',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # --- Static TF: chassis -> lidar frame ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_chassis_to_lidar',
            output='screen',
            arguments=[
                lidar_x, lidar_y, lidar_z,
                lidar_roll, lidar_pitch, lidar_yaw,
                'newcar/chassis', 'newcar/lidar_link/lidar'
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # --- Odom -> TF broadcaster (you need this for AMCL) ---
        # This assumes you have a node/executable in your package named "odom_to_tf"
        # If you haven't made it a package executable yet, tell me and I’ll show you how.
        Node(
            package='mymasterproject',   # <-- CHANGE to your package that contains odom_to_tf
            executable='odom_to_tf',     # <-- CHANGE if your executable name differs
            name='odom_to_tf',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'odom_topic': '/model/newcar/odometry'
            }],
        ),

        # --- Map server (your exact command) ---
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
                'frame_id': 'map'
            }],
        ),

        # --- AMCL ---
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'tf_broadcast': True,
                'global_frame_id': 'map',
                'odom_frame_id': 'newcar/odom',
                'base_frame_id': 'newcar/chassis',
                'scan_topic': '/scan',
            }],
        ),

        # --- Lifecycle manager: autostart map_server + amcl ---
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }],
        ),
    ])
