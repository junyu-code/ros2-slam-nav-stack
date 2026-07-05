#!/usr/bin/env bash
set -euo pipefail

# 清理本工作区常见的 ROS/Gazebo/RViz 残留进程。
pkill -TERM -f "slam_nav_simulation|slam_nav_bringup|fastlio_mapping|async_slam_toolbox_node|pointcloud_to_laserscan_node|gzserver|gzclient|rviz2" 2>/dev/null || true
sleep 1
pkill -KILL -f "slam_nav_simulation|slam_nav_bringup|fastlio_mapping|async_slam_toolbox_node|pointcloud_to_laserscan_node|gzserver|gzclient|rviz2" 2>/dev/null || true

ros2 daemon stop 2>/dev/null || true
rm -f /dev/shm/fastrtps_* 2>/dev/null || true

echo "[clean] done"
