
# mymasterproject

ROS 2 package for simulating a vehicle in Gazebo with full bringup.

## Requirements
- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

## Build
```bash
colcon build --packages-select mymasterproject
source install/setup.bash
## RUN
ros2 launch mymasterproject newcar_all.launch.py
