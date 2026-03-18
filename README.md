# My Master Project — Autonomous Navigation with a Custom Car Robot

A ROS2 workspace for autonomous navigation of a custom ackermann-steering car robot in Gazebo simulation, featuring pedestrian avoidance, path planning, and localization.

---

## Package Overview

### `mymasterproject`
The main package containing all nodes, launch files, and configuration for the autonomous navigation stack.

- **`scripts/`** — Core Python nodes:
  - `planner_node.py` — Path planning node
  - `dwa_local_planner.py` — DWA local planner
  - `controller_node.py` — Vehicle controller
  - `odom_to_tf.py` — Odometry to TF broadcaster
  - `actor_safety_node.py` — Pedestrian safety node
  - `nav_benchmark.py` — Navigation benchmarking tool
  - `local_costmap_generator.py` — Local costmap generation

- **`launch/`** — Launch files:
  - `newcar_launch.py` — Main launch file for the full navigation stack

- **`config/`** — Configuration files:
  - `nav2_params.yaml` — Nav2 navigation parameters
  - `ekf.yaml` — Extended Kalman Filter config

- **`actor_odom/`** — Pedestrian tracking nodes using pose arrays and TF

- **`maps/`** — Map files for localization

- **`models/`** — Gazebo actor models (e.g. pedestrians)

- **`src/world/`** — World and robot SDF/URDF files

---

### `robot_description`
URDF/Xacro robot description package for the custom ackermann car.

- **`description/`** — Xacro files defining the robot model
- **`launch/`** — Launch files for robot state publisher and simulation
- **`config/`** — Controller and sensor configs
- **`worlds/`** — Gazebo world files

---

### `actor_pose_ros2`
C++ package for publishing actor poses and moving waypoints in Gazebo simulation.

---

### `maze`
Assets and path planning scripts used for early maze-solving experiments.

- A* and RRT path planners
- Maze world SDF files
- Coordinate CSV files

---

### `ekf`
Standalone EKF configuration file for robot localization.

---

### `demo`
Simple demo SDF and URDF files used for early testing.
