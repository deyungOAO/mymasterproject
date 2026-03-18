import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # Specify the name of the package and path to xacro file within the package
    pkg_share = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_share, 'urdf', 'building_robot.urdf.xacro')


    # Use xacro to process the file
    robot_description_raw = xacro.process_file(xacro_file).toxml()


    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': True}] # add other parameters here if required
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'new_car'],
                    output='screen')

    # Run the node
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])



#--updated remove--
# from launch import LaunchDescription
# from launch.actions import ExecuteProcess
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # Get the path to the robot_description package
#     robot_description_path = get_package_share_directory('robot_description')
    
#     # Specify the SDF world file
#     world_file_name = 'building_robot_debug.sdf'  # Replace with your actual world file name
#     world_path = os.path.join(robot_description_path, 'worlds', world_file_name)
    
#     # Check if the world file exists
#     if not os.path.exists(world_path):
#         raise FileNotFoundError(f"World file not found: {world_path}")

#     return LaunchDescription([
#         # Launch Gazebo with the specified world
#         ExecuteProcess(
#             cmd=['ign', 'gazebo', '--verbose', world_path],
#             output='screen'
#         )
#     ])
