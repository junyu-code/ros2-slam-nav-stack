cmake_minimum_required(VERSION 3.10)
project(slam_nav_bringup)

find_package(ament_cmake REQUIRED)

foreach(_dir behavior_tree config launch map report_assets rviz)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_dir}")
    install(DIRECTORY
      ${_dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

install(PROGRAMS
  scripts/amcl_convergence_monitor.py
  scripts/auto_explore_mapping.py
  scripts/diagnose_runtime.py
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
