from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')

    # point directly to your src tree
    my_pkg_src_dir = os.path.join(
        os.path.expanduser('~'),
        'ros2_newws',
        'src',
        'mymasterproject'
    )

    map_file = os.path.join(my_pkg_src_dir, 'maps', 'map.yaml')
    params_file = os.path.join(my_pkg_src_dir, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'use_collision_monitor': 'false',
                'map': map_file,
                'params_file': params_file,
            }.items()
        )
    ])
