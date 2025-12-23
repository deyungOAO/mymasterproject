from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = 'mymasterproject'
    world = PathJoinSubstitution([get_package_share_directory(pkg), 'worlds', 'campus_world.sdf'])
    nav2_params = PathJoinSubstitution([get_package_share_directory(pkg), 'config', 'nav2_params.yaml'])

    # 1) Gazebo (gz sim)
    gz = ExecuteProcess(cmd=['gz', 'sim', world], output='screen')

    # 2) Bridges (laser -> /scan ; (you already had this) )
    bridge_scan = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan]']
    )

    # 3) People poses -> point cloud
    people_cloud = Node(
        package=pkg,
        executable='posearray_to_cloud.py',
        name='posearray_to_cloud',
        output='screen',
        parameters=[{
            'input_topic': '/world/campus_world/dynamic_pose/info',
            'output_topic': '/people_cloud',
            'frame_id': 'world'
        }]
    )

    # 4) Static TF world->map (AMCL/MapServer uses map; Gazebo uses world)
    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','world','map'],
        output='screen'
    )

    # 5) Nav2 bringup (map_server, AMCL, planner/controller, BT nav)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params
        }.items()
    )

    # 6) RViz (optional—use your rviz config if you have one)
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])]
    )

    return LaunchDescription([gz, bridge_scan, people_cloud, static_tf, nav2_bringup, rviz])
