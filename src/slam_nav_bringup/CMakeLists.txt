cmake_minimum_required(VERSION 3.10)
project(slam_nav_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  map
  report_assets
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
