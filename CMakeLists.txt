#!/usr/bin/env python3
cmake_minimum_required(VERSION 3.8)
project(mymasterproject)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/posearray_to_cloud.py
  scripts/odom_to_tf.py
  scripts/odom_tf_broadcaster.py
  scripts/pose_to_odom.py
  scripts/robot_pose_node.py
  scripts/planner_node.py
  scripts/controller_node.py
  mymasterproject/actor_robot_tf_broadcaster.py
  scripts/simple_inflation_costmap.py
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
