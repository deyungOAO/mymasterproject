import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros
import launch
import xacro


def generate_launch_description():

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path,'description','new_car_master.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz/robot_description.rviz')

    # Define node parameters
    # Create a robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro', ' ', LaunchConfiguration('model')])}]
    )

    #fakes joint-states to test urdf
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[xacro_file]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Launch!
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=xacro_file,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_file,
                                            description='Absolute path to rviz config file'),    

        robot_state_publisher,
        joint_state_publisher_node,
        rviz_node,
    ])