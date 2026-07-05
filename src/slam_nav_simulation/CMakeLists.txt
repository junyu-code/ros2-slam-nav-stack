cmake_minimum_required(VERSION 3.10)
project(slam_nav_simulation)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GAZEBO REQUIRED gazebo)

add_library(dynamic_obstacle_plugin SHARED
  src/dynamic_obstacle_plugin.cpp
)
target_include_directories(dynamic_obstacle_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(dynamic_obstacle_plugin PRIVATE
  ${GAZEBO_LIBRARY_DIRS}
)
target_link_libraries(dynamic_obstacle_plugin
  ${GAZEBO_LIBRARIES}
)

install(DIRECTORY
  launch
  meshes
  rviz
  urdf
  world
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  dynamic_obstacle_plugin
  LIBRARY DESTINATION lib
)

ament_package()
