cmake_minimum_required(VERSION 3.10)
project(slam_nav_simulation)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig QUIET)

if(PkgConfig_FOUND)
  pkg_check_modules(GAZEBO QUIET gazebo)
endif()

if(GAZEBO_FOUND)
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

  install(TARGETS
    dynamic_obstacle_plugin
    LIBRARY DESTINATION lib
  )
else()
  message(WARNING "Gazebo was not found; dynamic_obstacle_plugin will not be built.")
endif()

foreach(_dir launch meshes rviz urdf world)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(DIRECTORY
      ${_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

ament_package()
