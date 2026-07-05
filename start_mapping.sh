#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

exec ros2 launch slam_nav_bringup mapping.launch.py use_sim_time:=true rviz:=true
