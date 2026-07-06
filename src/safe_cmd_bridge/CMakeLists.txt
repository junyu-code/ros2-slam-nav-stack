cmake_minimum_required(VERSION 3.8)
project(safe_cmd_bridge)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/safe_cmd_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
