cmake_minimum_required(VERSION 3.8)
project(actor_pose_ros2)

# ── Compiler settings ─────────────────────────────────────────────────────────
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

# ── ROS 2 packages ────────────────────────────────────────────────────────────
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)   # ← was missing
find_package(geometry_msgs  REQUIRED)
find_package(std_msgs       REQUIRED)
find_package(tf2_ros        REQUIRED)

# ── Gazebo packages ───────────────────────────────────────────────────────────
find_package(gz-sim8        REQUIRED)
find_package(gz-plugin2     REQUIRED)
find_package(gz-math7       REQUIRED)

# ── actor_pose_ros2 plugin ────────────────────────────────────────────────────
add_library(actor_pose_ros2 SHARED
  src/actor_pose_ros2.cpp
)
ament_target_dependencies(actor_pose_ros2
  rclcpp
  geometry_msgs
  tf2_ros
)
target_link_libraries(actor_pose_ros2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

# ── model_waypoint_mover plugin ───────────────────────────────────────────────
add_library(model_waypoint_mover SHARED
  src/model_waypoint_mover.cpp
)
ament_target_dependencies(model_waypoint_mover
  rclcpp         # ← this adds the include dirs and links rclcpp
  std_msgs
)
target_link_libraries(model_waypoint_mover
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  actor_pose_ros2
  model_waypoint_mover
  LIBRARY DESTINATION lib
)

ament_package()
