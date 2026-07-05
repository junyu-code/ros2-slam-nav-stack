#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel
