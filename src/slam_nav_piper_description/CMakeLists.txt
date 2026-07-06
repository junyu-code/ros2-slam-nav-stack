cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  rviz
  scripts
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/piper_description_builder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
