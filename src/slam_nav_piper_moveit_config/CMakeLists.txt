cmake_minimum_required(VERSION 3.10)
project(slam_nav_piper_moveit_config)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/piper_fake_joint_state_publisher.py
  scripts/piper_moveit_config_audit.py
  scripts/piper_plan_smoke_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
