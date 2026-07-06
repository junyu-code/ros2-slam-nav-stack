cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/piper_official_frame_audit.py
  scripts/piper_preflight_check.py
  scripts/piper_real_readiness_report.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
