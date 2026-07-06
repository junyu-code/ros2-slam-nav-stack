cmake_minimum_required(VERSION 3.10)
project(slam_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  behavior_tree
  config
  launch
  map
  report_assets
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/diagnose_runtime.py
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
