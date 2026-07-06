cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_learning)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/grasp_candidate_ranker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
