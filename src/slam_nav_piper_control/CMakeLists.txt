cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/piper_control_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
