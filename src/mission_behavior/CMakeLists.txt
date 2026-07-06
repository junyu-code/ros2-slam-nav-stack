cmake_minimum_required(VERSION 3.10)
project(mission_behavior)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/mission_behavior_node.py
  scripts/mission_piper_pick_place_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_tree
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
