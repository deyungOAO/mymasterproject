import os
import launch
import launch_ros.actions

def generate_launch_description():
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.join('/home/ubuntu/AGV_ws/src')

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_description',
            executable='parking_spot_detection.py',
            name='parking_spot_detection',
            output='screen',
            parameters=[{'use_sim_time': True}]  
        ),
        launch_ros.actions.Node(
            package='robot_description',
            executable='parallel_parking.py',
            name='parallel_parking',
            output='screen',
            parameters=[{'use_sim_time': True}]  
        ),
    ])
