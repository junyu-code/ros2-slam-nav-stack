#!/usr/bin/env bash

# Shared real-robot sensor bootstrap for entries that consume FAST-LIO output.
# It only stops processes started by this helper; already-running drivers are left alone.

REAL_SENSOR_STARTED_PIDS=()
REAL_SENSOR_CLEANUP_INSTALLED=false
REAL_SENSOR_LIVOX_LAST_PID=""
REAL_SENSOR_FILTER_LAST_PID=""
REAL_SENSOR_LIDAR_READY=false
REAL_SENSOR_RAW_IMU_READY=false
REAL_SENSOR_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

real_sensor_warn_host_ip() {
  local host_ip="${REAL_SENSOR_LIDAR_HOST_IP:-${LIDAR_HOST_IP:-192.168.1.150}}"
  local iface="${REAL_SENSOR_LIDAR_IFACE:-${LIDAR_IFACE:-}}"
  local ip_addr_output

  if ! command -v ip >/dev/null 2>&1; then
    return
  fi

  if [[ -n "${iface}" ]]; then
    ip_addr_output="$(ip -4 addr show dev "${iface}" 2>/dev/null || true)"
  else
    ip_addr_output="$(ip -4 addr show 2>/dev/null || true)"
  fi

  if grep -Fq "${host_ip}/" <<<"${ip_addr_output}"; then
    echo "[real-sensors] host LiDAR IP ready: ${host_ip}"
    return
  fi

  if [[ -n "${iface}" ]]; then
    echo "[real-sensors] WARN: ${iface} 未检测到 ${host_ip}/24；MID360 配置需要主机 IP 为 ${host_ip}" >&2
    echo "[real-sensors] WARN: 如确认使用 ${iface}，先执行: sudo ip addr add ${host_ip}/24 dev ${iface}" >&2
  else
    echo "[real-sensors] WARN: 未在任意网卡检测到 ${host_ip}/24；MID360 配置需要主机 IP 为 ${host_ip}" >&2
    echo "[real-sensors] WARN: 可用 LIDAR_IFACE=<网卡名> 指定网卡，或先给雷达网卡配置该 IP" >&2
  fi
}

real_sensor_topic_has_publisher() {
  local topic="$1"
  local info
  info="$(ROS2CLI_DISABLE_DAEMON=1 timeout "${REAL_SENSOR_TOPIC_CHECK_TIMEOUT:-1s}" ros2 topic info "${topic}" -v 2>/dev/null || true)"
  if grep -Eq '^Publisher count: [1-9][0-9]*' <<<"${info}"; then
    return 0
  fi
  ROS2CLI_DISABLE_DAEMON=1 timeout "${REAL_SENSOR_TOPIC_CHECK_TIMEOUT:-1s}" ros2 topic list 2>/dev/null | grep -qx "${topic}"
}

real_sensor_start_livox_driver() {
  real_sensor_warn_host_ip
  echo "[real-sensors] starting Livox MID360 driver"
  setsid "${REAL_SENSOR_SCRIPT_DIR}/start_livox_mid360_go2.sh" &
  REAL_SENSOR_LIVOX_LAST_PID="$!"
  REAL_SENSOR_STARTED_PIDS+=("${REAL_SENSOR_LIVOX_LAST_PID}")
}

real_sensor_start_imu_filter() {
  local raw_imu_topic="${REAL_SENSOR_RAW_IMU_TOPIC:-/livox/imu}"
  echo "[real-sensors] starting IMU complementary filter"
  setsid ros2 launch imu_complementary_filter complementary_filter.launch.py \
    use_sim_time:="${USE_SIM_TIME:-false}" \
    raw_imu_topic:="${raw_imu_topic}" &
  REAL_SENSOR_FILTER_LAST_PID="$!"
  REAL_SENSOR_STARTED_PIDS+=("${REAL_SENSOR_FILTER_LAST_PID}")
}

real_sensor_stop_pid() {
  local pid="$1"
  if [[ -n "${pid}" ]]; then
    kill -TERM -- "-${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${pid}" 2>/dev/null || true
  fi
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

real_sensor_wait_for_livox_pair() {
  local lidar_topic="$1"
  local raw_imu_topic="$2"
  local require_raw_imu="$3"
  local timeout_sec="$4"
  local end_time=$((SECONDS + timeout_sec))

  REAL_SENSOR_LIDAR_READY=false
  REAL_SENSOR_RAW_IMU_READY=false

  while (( SECONDS < end_time )); do
    if [[ "${REAL_SENSOR_LIDAR_READY}" == "false" ]] && real_sensor_topic_has_publisher "${lidar_topic}"; then
      REAL_SENSOR_LIDAR_READY=true
      echo "[real-sensors] Livox LiDAR ready: ${lidar_topic}"
    fi
    if [[ "${REAL_SENSOR_RAW_IMU_READY}" == "false" ]] && real_sensor_topic_has_publisher "${raw_imu_topic}"; then
      REAL_SENSOR_RAW_IMU_READY=true
      echo "[real-sensors] Livox raw IMU ready: ${raw_imu_topic}"
    fi

    if [[ "${REAL_SENSOR_LIDAR_READY}" == "true" &&
          ( "${require_raw_imu}" != "true" || "${REAL_SENSOR_RAW_IMU_READY}" == "true" ) ]]; then
      return 0
    fi
    sleep 1
  done

  if [[ "${REAL_SENSOR_LIDAR_READY}" != "true" ]]; then
    echo "[real-sensors] WARN: Livox LiDAR not publishing after ${timeout_sec}s: ${lidar_topic}" >&2
  fi
  if [[ "${require_raw_imu}" == "true" && "${REAL_SENSOR_RAW_IMU_READY}" != "true" ]]; then
    echo "[real-sensors] WARN: Livox raw IMU not publishing after ${timeout_sec}s: ${raw_imu_topic}" >&2
  fi
  return 1
}

real_sensor_wait_for_publisher() {
  local topic="$1"
  local label="$2"
  local timeout_sec="${3:-15}"
  local end_time=$((SECONDS + timeout_sec))

  while (( SECONDS < end_time )); do
    if real_sensor_topic_has_publisher "${topic}"; then
      echo "[real-sensors] ${label} ready: ${topic}"
      return 0
    fi
    sleep 1
  done

  echo "[real-sensors] WARN: ${label} not publishing after ${timeout_sec}s: ${topic}" >&2
  return 1
}

real_sensor_should_start() {
  local setting="$1"
  local topic="$2"

  case "${setting}" in
    true|1|yes|on)
      return 0
      ;;
    false|0|no|off)
      return 1
      ;;
    auto|"")
      ! real_sensor_topic_has_publisher "${topic}"
      return
      ;;
    *)
      echo "[real-sensors] WARN: unknown setting '${setting}', treating it as auto" >&2
      ! real_sensor_topic_has_publisher "${topic}"
      return
      ;;
  esac
}

real_sensor_cleanup() {
  local pid

  for pid in "${REAL_SENSOR_STARTED_PIDS[@]:-}"; do
    real_sensor_stop_pid "${pid}"
  done
}

real_sensor_install_cleanup() {
  if [[ "${REAL_SENSOR_CLEANUP_INSTALLED}" == "true" ]]; then
    return
  fi

  trap real_sensor_cleanup EXIT INT TERM
  REAL_SENSOR_CLEANUP_INSTALLED=true
}

start_real_lidar_inputs() {
  local start_livox="${REAL_SENSOR_START_LIVOX:-auto}"
  local start_imu_filter="${REAL_SENSOR_START_IMU_FILTER:-auto}"
  local strict="${REAL_SENSOR_STRICT:-false}"
  local lidar_topic="${REAL_SENSOR_LIDAR_TOPIC:-/livox/lidar}"
  local raw_imu_topic="${REAL_SENSOR_RAW_IMU_TOPIC:-/livox/imu}"
  local filtered_imu_topic="${REAL_SENSOR_FILTERED_IMU_TOPIC:-/imu/data}"
  local livox_timeout="${REAL_SENSOR_LIVOX_TIMEOUT:-10}"
  local imu_timeout="${REAL_SENSOR_IMU_TIMEOUT:-5}"
  local livox_attempts="${REAL_SENSOR_LIVOX_ATTEMPTS:-2}"
  local imu_filter_attempts="${REAL_SENSOR_IMU_FILTER_ATTEMPTS:-2}"
  local need_imu_filter=false
  local start_livox_now=false
  local filtered_imu_ready=false
  local attempt
  local failures=0

  real_sensor_install_cleanup
  if real_sensor_should_start "${start_imu_filter}" "${filtered_imu_topic}"; then
    need_imu_filter=true
  fi

  if real_sensor_should_start "${start_livox}" "${lidar_topic}"; then
    start_livox_now=true
  else
    echo "[real-sensors] using existing or disabled Livox driver"
  fi

  if [[ "${start_livox_now}" == "true" ]]; then
    for (( attempt = 1; attempt <= livox_attempts; attempt++ )); do
      real_sensor_start_livox_driver
      if real_sensor_wait_for_livox_pair "${lidar_topic}" "${raw_imu_topic}" "${need_imu_filter}" "${livox_timeout}"; then
        break
      fi
      if (( attempt < livox_attempts )); then
        echo "[real-sensors] Livox driver not ready; restarting attempt $((attempt + 1))/${livox_attempts}" >&2
        real_sensor_stop_pid "${REAL_SENSOR_LIVOX_LAST_PID}"
      fi
    done
  else
    real_sensor_wait_for_livox_pair "${lidar_topic}" "${raw_imu_topic}" "${need_imu_filter}" "${livox_timeout}" || true
  fi

  if [[ "${REAL_SENSOR_LIDAR_READY}" != "true" ]]; then
    failures=$((failures + 1))
  fi
  if [[ "${need_imu_filter}" == "true" && "${REAL_SENSOR_RAW_IMU_READY}" != "true" ]]; then
    failures=$((failures + 1))
  fi

  if [[ "${need_imu_filter}" == "true" ]]; then
    if [[ "${REAL_SENSOR_RAW_IMU_READY}" == "true" ]]; then
      for (( attempt = 1; attempt <= imu_filter_attempts; attempt++ )); do
        real_sensor_start_imu_filter
        if real_sensor_wait_for_publisher "${filtered_imu_topic}" "filtered IMU" "${imu_timeout}"; then
          filtered_imu_ready=true
          break
        fi
        if (( attempt < imu_filter_attempts )); then
          echo "[real-sensors] IMU filter not ready; restarting attempt $((attempt + 1))/${imu_filter_attempts}" >&2
          real_sensor_stop_pid "${REAL_SENSOR_FILTER_LAST_PID}"
        fi
      done
    else
      echo "[real-sensors] WARN: raw IMU missing; skip IMU filter start" >&2
    fi
  else
    echo "[real-sensors] using existing or disabled IMU filter"
    if real_sensor_wait_for_publisher "${filtered_imu_topic}" "filtered IMU" "${imu_timeout}"; then
      filtered_imu_ready=true
    fi
  fi

  if [[ "${filtered_imu_ready}" != "true" ]]; then
    failures=$((failures + 1))
  fi

  if (( failures > 0 )); then
    echo "[real-sensors] ERROR: 实机传感器链路未就绪，缺失 ${failures} 个关键发布者。" >&2
    echo "[real-sensors]        建图不需要深度相机；这里只需要 MID360 点云和 IMU。" >&2
    echo "[real-sensors]        driver 若显示 Init success 但没有 /livox/* publisher，通常是 SDK 未发现雷达或未收到数据包。" >&2
    echo "[real-sensors]        请运行: ./run.sh livox-debug" >&2
    if [[ "${strict}" =~ ^(true|1|yes|on)$ ]]; then
      return 1
    fi
  fi
}
