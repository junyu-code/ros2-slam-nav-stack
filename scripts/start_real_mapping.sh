#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."

source "${SCRIPT_DIR}/setup_workspace_env.sh"
source "${SCRIPT_DIR}/real_sensor_inputs.sh"
set -u

export USE_SIM_TIME="${USE_SIM_TIME:-false}"
export REAL_SENSOR_STRICT="${REAL_SENSOR_STRICT:-true}"
export REAL_SENSOR_LIVOX_ATTEMPTS="${REAL_SENSOR_LIVOX_ATTEMPTS:-2}"
export REAL_SENSOR_IMU_FILTER_ATTEMPTS="${REAL_SENSOR_IMU_FILTER_ATTEMPTS:-2}"
export REAL_SENSOR_LIVOX_TIMEOUT="${REAL_SENSOR_LIVOX_TIMEOUT:-15}"
export REAL_SENSOR_IMU_TIMEOUT="${REAL_SENSOR_IMU_TIMEOUT:-8}"

host_ip="${LIDAR_HOST_IP:-192.168.1.150}"
lidar_ip="${LIDAR_IP:-192.168.1.143}"
lidar_ip_candidates="${LIDAR_IP_CANDIDATES:-${lidar_ip}}"
iface="${LIDAR_IFACE:-}"
auto_clean="${REAL_MAPPING_CLEAN:-true}"
auto_net="${REAL_MAPPING_AUTO_NET:-true}"
lidar_topic="${REAL_SENSOR_LIDAR_TOPIC:-/livox/lidar}"
raw_imu_topic="${REAL_SENSOR_RAW_IMU_TOPIC:-/livox/imu}"
filtered_imu_topic="${REAL_SENSOR_FILTERED_IMU_TOPIC:-/imu/data}"
started_pids=()

ip_has_host_address() {
  ip -4 addr show 2>/dev/null | grep -Fq "${host_ip}/"
}

prepare_lidar_network() {
  if ! command -v ip >/dev/null 2>&1; then
    return
  fi

  if ip_has_host_address; then
    echo "[real-mapping] LiDAR host IP ready: ${host_ip}"
    return
  fi

  if [[ "${auto_net}" != "true" ]]; then
    echo "[real-mapping] WARN: missing ${host_ip}/24; auto network config disabled" >&2
    return
  fi

  if [[ -z "${iface}" ]]; then
    iface="$(ip route get "${lidar_ip}" 2>/dev/null | awk '/ dev / {for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' || true)"
  fi

  if [[ -z "${iface}" ]]; then
    echo "[real-mapping] WARN: missing ${host_ip}/24 and no LIDAR_IFACE was provided" >&2
    echo "[real-mapping] WARN: run with: LIDAR_IFACE=<网卡名> ./run.sh real-mapping" >&2
    return
  fi

  echo "[real-mapping] adding ${host_ip}/24 to ${iface}"
  sudo ip addr add "${host_ip}/24" dev "${iface}" || true
  sudo ip link set "${iface}" up || true

  if ip_has_host_address; then
    echo "[real-mapping] LiDAR host IP ready: ${host_ip}"
  else
    echo "[real-mapping] WARN: failed to configure ${host_ip}/24 on ${iface}" >&2
  fi
}

detect_lidar_iface() {
  local target_ip="$1"
  local neigh_iface

  if [[ -n "${iface}" ]]; then
    echo "${iface}"
    return
  fi

  neigh_iface="$(ip neigh show "${target_ip}" 2>/dev/null | awk '/lladdr/ {for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' || true)"
  if [[ -n "${neigh_iface}" ]]; then
    echo "${neigh_iface}"
    return
  fi

  ip route get "${target_ip}" 2>/dev/null | awk '/ dev / {for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' || true
}

prepare_lidar_route() {
  local target_ip="$1"
  local target_iface
  local route_iface

  if ! command -v ip >/dev/null 2>&1; then
    return
  fi

  target_iface="$(detect_lidar_iface "${target_ip}")"
  route_iface="$(ip route get "${target_ip}" 2>/dev/null | awk '/ dev / {for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' || true)"

  if [[ -z "${target_iface}" ]]; then
    return
  fi

  if [[ "${route_iface}" == "${target_iface}" ]] && ip -4 addr show dev "${target_iface}" | grep -Fq "${host_ip}"; then
    return
  fi

  echo "[real-mapping] routing LiDAR ${target_ip} through ${target_iface}"
  if ! ip -4 addr show dev "${target_iface}" | grep -Fq "${host_ip}"; then
    sudo ip addr add "${host_ip}/32" dev "${target_iface}" || true
  fi
  sudo ip link set "${target_iface}" up || true
  sudo ip route replace "${target_ip}/32" dev "${target_iface}" src "${host_ip}"
}

stop_started_processes() {
  local pid
  for pid in "${started_pids[@]:-}"; do
    real_sensor_stop_pid "${pid}"
  done
}

start_background_process() {
  local label="$1"
  shift
  echo "[real-mapping] starting ${label}"
  setsid "$@" &
  started_pids+=("$!")
}

last_started_pid() {
  local count="${#started_pids[@]}"
  if (( count == 0 )); then
    return 1
  fi
  echo "${started_pids[$((count - 1))]}"
}

remove_last_started_pid() {
  local count="${#started_pids[@]}"
  if (( count > 0 )); then
    unset "started_pids[$((count - 1))]"
  fi
}

trap stop_started_processes EXIT INT TERM

echo "[real-mapping] one-shot real mapping startup"
echo "[real-mapping] LiDAR ${lidar_ip}, host ${host_ip}, use_sim_time=${USE_SIM_TIME}"

if [[ "${auto_clean}" == "true" ]]; then
  echo "[real-mapping] cleaning stale mapping/navigation processes"
  "${SCRIPT_DIR}/clean.sh"
  pkill -TERM -f "ros2 launch livox_ros_driver2|ros2 launch imu_complementary_filter" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "ros2 launch livox_ros_driver2|ros2 launch imu_complementary_filter" 2>/dev/null || true
fi

prepare_lidar_network

for candidate_ip in ${lidar_ip_candidates}; do
  export LIDAR_IP="${candidate_ip}"
  echo "[real-mapping] trying MID360 IP ${LIDAR_IP}"
  prepare_lidar_route "${LIDAR_IP}"
  for (( attempt = 1; attempt <= REAL_SENSOR_LIVOX_ATTEMPTS; attempt++ )); do
    echo "[real-mapping] 1/3 start ./run.sh livox-mid360 (${attempt}/${REAL_SENSOR_LIVOX_ATTEMPTS})"
    start_background_process "Livox MID360 driver" "${SCRIPT_DIR}/start_livox_mid360_go2.sh"
    if real_sensor_wait_for_livox_pair "${lidar_topic}" "${raw_imu_topic}" true "${REAL_SENSOR_LIVOX_TIMEOUT}"; then
      break 2
    fi
    if (( attempt < REAL_SENSOR_LIVOX_ATTEMPTS )); then
      echo "[real-mapping] Livox not ready; restarting driver" >&2
      real_sensor_stop_pid "$(last_started_pid || true)"
      remove_last_started_pid
    fi
  done
done

if [[ "${REAL_SENSOR_LIDAR_READY}" != "true" || "${REAL_SENSOR_RAW_IMU_READY}" != "true" ]]; then
  last_pid="$(last_started_pid || true)"
  if [[ -n "${last_pid}" ]] && kill -0 "${last_pid}" 2>/dev/null; then
    echo "[real-mapping] WARN: ROS graph did not report /livox topics yet, but Livox driver process is running; continuing."
    REAL_SENSOR_LIDAR_READY=true
    REAL_SENSOR_RAW_IMU_READY=true
  fi
fi

if [[ "${REAL_SENSOR_LIDAR_READY}" != "true" || "${REAL_SENSOR_RAW_IMU_READY}" != "true" ]]; then
  echo "[real-mapping] ERROR: MID360 没有发布 ${lidar_topic} / ${raw_imu_topic}" >&2
  echo "[real-mapping] 建图只需要 MID360，不需要深度相机。现在卡在 Livox 发现/数据接收阶段。" >&2
  echo "[real-mapping] 运行 ./run.sh livox-debug 查看网络和 driver 状态。" >&2
  exit 1
fi

for (( attempt = 1; attempt <= REAL_SENSOR_IMU_FILTER_ATTEMPTS; attempt++ )); do
  echo "[real-mapping] 2/3 start ./run.sh imu-filter (${attempt}/${REAL_SENSOR_IMU_FILTER_ATTEMPTS})"
  start_background_process "IMU complementary filter" "${SCRIPT_DIR}/start_imu_filter.sh"
  if real_sensor_wait_for_publisher "${filtered_imu_topic}" "filtered IMU" "${REAL_SENSOR_IMU_TIMEOUT}"; then
    break
    fi
    if (( attempt < REAL_SENSOR_IMU_FILTER_ATTEMPTS )); then
      echo "[real-mapping] IMU filter not ready; restarting filter" >&2
      real_sensor_stop_pid "$(last_started_pid || true)"
      remove_last_started_pid
    fi
  done

if ! real_sensor_topic_has_publisher "${filtered_imu_topic}"; then
  last_pid="$(last_started_pid || true)"
  if [[ -n "${last_pid}" ]] && kill -0 "${last_pid}" 2>/dev/null; then
    echo "[real-mapping] WARN: ROS graph did not report ${filtered_imu_topic} yet, but IMU filter process is running; continuing."
  else
    echo "[real-mapping] ERROR: IMU filter 没有发布 ${filtered_imu_topic}" >&2
    exit 1
  fi
fi

echo "[real-mapping] 3/3 start mapping core: FAST-LIO + /scan + slam_toolbox + RViz"
ros2 launch slam_nav_bringup mapping.launch.py \
  use_sim_time:="${USE_SIM_TIME}" \
  rviz:=true \
  "$@"
