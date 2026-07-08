#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
source "${SCRIPT_DIR}/real_sensor_inputs.sh"
set -u

export REAL_SENSOR_STRICT="${REAL_SENSOR_STRICT:-true}"
start_real_lidar_inputs
ros2 launch slam_nav_bringup mapping.launch.py use_sim_time:="${USE_SIM_TIME:-false}" rviz:=true "$@"
