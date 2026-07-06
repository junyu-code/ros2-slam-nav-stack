cmake_minimum_required(VERSION 3.10)
project(perception_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(adaptive_cloud_filter src/adaptive_cloud_filter.cpp)
target_compile_features(adaptive_cloud_filter PUBLIC cxx_std_17)
ament_target_dependencies(adaptive_cloud_filter
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  adaptive_cloud_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
