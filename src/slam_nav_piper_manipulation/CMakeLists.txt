cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_manipulation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/piper_task_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
