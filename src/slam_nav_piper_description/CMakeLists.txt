cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_description)

find_package(ament_cmake REQUIRED)

foreach(_dir launch rviz scripts urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(DIRECTORY
      ${_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

install(PROGRAMS
  scripts/piper_description_builder.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
