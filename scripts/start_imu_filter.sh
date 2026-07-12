#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

raw_imu_topic="${REAL_SENSOR_RAW_IMU_TOPIC:-/livox/imu}"

exec ros2 launch imu_complementary_filter complementary_filter.launch.py \
  use_sim_time:="${USE_SIM_TIME:-false}" \
  raw_imu_topic:="${raw_imu_topic}" \
  "$@"
