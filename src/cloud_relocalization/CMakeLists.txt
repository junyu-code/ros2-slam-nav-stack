cmake_minimum_required(VERSION 3.10)
project(cloud_relocalization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree registration)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# 直接构建工作区中从上游拉取的 small_gicp，避免依赖系统预安装版本。
set(SMALL_GICP_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../external/small_gicp" CACHE PATH
  "small_gicp 源码目录")
if(NOT EXISTS "${SMALL_GICP_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "small_gicp source is missing. Run: git clone https://github.com/koide3/small_gicp.git external/small_gicp")
endif()
set(BUILD_HELPER ON CACHE BOOL "Build small_gicp helper library" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "Disable small_gicp tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Disable small_gicp examples" FORCE)
add_subdirectory("${SMALL_GICP_SOURCE_DIR}" "${CMAKE_CURRENT_BINARY_DIR}/small_gicp")

add_executable(icp_relocalization_node src/icp_relocalization_node.cpp)
target_compile_features(icp_relocalization_node PUBLIC cxx_std_17)
target_include_directories(icp_relocalization_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(icp_relocalization_node ${PCL_LIBRARIES} small_gicp)
ament_target_dependencies(icp_relocalization_node
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

add_executable(bnb_localization_node src/bnb_localization_node.cpp)
target_compile_features(bnb_localization_node PUBLIC cxx_std_17)
target_include_directories(bnb_localization_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(bnb_localization_node ${PCL_LIBRARIES})
ament_target_dependencies(bnb_localization_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
)

install(TARGETS
  bnb_localization_node
  icp_relocalization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/relocalization_amcl_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
