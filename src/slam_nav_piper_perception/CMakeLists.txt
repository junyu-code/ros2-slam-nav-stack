cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_perception)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/arm_camera_fake_node.py
  scripts/target_pose_estimator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
