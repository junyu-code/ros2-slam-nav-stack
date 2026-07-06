cmake_minimum_required(VERSION 3.8)
project(localization_guard)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/localization_guard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
