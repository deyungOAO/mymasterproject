from setuptools import setup
from glob import glob
import os

package_name = 'mymasterproject'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] if os.path.isdir(package_name) else [],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('lib/' + package_name, ['src/people_to_cloud.py']),  # install as executable
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Nav2 + Gazebo people avoidance demo',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # if later you convert to a proper Python module:
            'people_to_cloud = people_to_cloud:main',
            'odom_tf_broadcaster = mymasterproject.odom_tf_broadcaster:main',
            'actor_robot_tf_broadcaster = mymasterproject.actor_robot_tf_broadcaster:main',
            'robot_pose_node = mymasterproject.robot_pose_node:main',
            'simple_planner_node = mymasterproject.planner_node:main',
            'simple_controller_node = mymasterproject.controller_node:main',
            'simple_inflation_costmap = mymasterproject.simple_inflation_costmap:main',
            'planner_node.py = mymasterproject.planner_node:main',
            'fast_a_star_planner.py = mymasterproject.fast_a_star_planner:main',
            'controller_node.py = mymasterproject.controller_node:main',
            'robot_pose_node.py = mymasterproject.robot_pose_node:main',
        ],
    },
)
