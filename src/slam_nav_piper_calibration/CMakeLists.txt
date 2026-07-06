cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_calibration)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/piper_hand_eye_config_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
