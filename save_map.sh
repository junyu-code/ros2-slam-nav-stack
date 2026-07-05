#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

mkdir -p src/slam_nav_bringup/map
name="${1:-nav_test_map}"
exec ros2 run nav2_map_server map_saver_cli -f "src/slam_nav_bringup/map/${name}"
