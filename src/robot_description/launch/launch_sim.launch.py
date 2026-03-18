import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_name = 'robot_description' 
    ros_gz_sim = get_package_share_directory('ros_gz_sim') 
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    #share directory to read stl file
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join('/home/ubuntu/AGV_ws/src')

    #check robot_state_publisher.launch.py to modify urdf/xacro
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','robot_state_publisher.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()

    )

    world = os.path.join(
        get_package_share_directory('robot_description'),
        'worlds',
        #uncomment to choose a world
        # 'test.world' #2 cars
        'test2.world' #3 cars, one cone
        #'depth_camera_callibration.world' #has walls to test the boundaries of depth camera for parking detection
        # 'empty.world'
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robot_description"),
            "config",
            "controllers.yaml",
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    ) 

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        launch_arguments={'gz_args': '-g -v4 '}.items()
    ) 

    gz_spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'car', '-allow_renaming', 'true'], 
    ) 

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",]
    )

    robot_ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ack_cont", "--param-file", robot_controllers],
    )

    teleop_twist_keyboard = Node(
        package ='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        remappings=[('keyboard/cmd_vel', '/ack_cont/reference_unstamped')] #remap topic
    )
    #relay the twist commands from parallel_parking.py to ackermann controller
    relay_parallel_parking = Node(
    package='topic_tools',
    executable='relay',
    arguments=['/cmd_vel', '/ack_cont/reference_unstamped'],
    output='screen'
    )

    # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(get_package_share_directory(package_name),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',]
    )

    #relay the ackermann steering odom to rviz odom
    #not necessary anymore since the EKF filter gives /tf
    relay_tf = Node(
    package='topic_tools',
    executable='relay',
    arguments=['/ack_cont/tf_odometry', '/tf'],
    output='screen'
    )

    #Extended Kalman Filter (EKF) reduce drift
    EKF_params = os.path.join(get_package_share_directory(package_name),'config','ekf.yaml')
    EKF = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{"use_sim_time": True}, EKF_params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),       
        rsp,
        gazebo_server, 
        gazebo_client,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        robot_ackermann_controller_spawner,
        gz_spawn_entity,
        # teleop_twist_keyboard,
        # relay_tf,
        relay_parallel_parking,
        ros_gz_bridge,
        EKF,
    ]) 
