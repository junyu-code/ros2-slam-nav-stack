cmake_minimum_required(VERSION 3.8)
project(slam_nav_operator)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(slam_nav_operator src/operator_main.cpp)

ament_target_dependencies(
  slam_nav_operator
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
)

target_link_libraries(slam_nav_operator Qt5::Widgets)

install(TARGETS slam_nav_operator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
