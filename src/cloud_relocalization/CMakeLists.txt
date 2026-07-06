cmake_minimum_required(VERSION 3.10)
project(cloud_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io registration)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(icp_relocalization_node src/icp_relocalization_node.cpp)
target_compile_features(icp_relocalization_node PUBLIC cxx_std_17)
target_include_directories(icp_relocalization_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(icp_relocalization_node ${PCL_LIBRARIES})
ament_target_dependencies(icp_relocalization_node
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

install(TARGETS
  icp_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
