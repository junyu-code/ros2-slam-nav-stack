#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

LIDAR_HOST_IP="${LIDAR_HOST_IP:-192.168.1.50}"
LIDAR_IP="${LIDAR_IP:-192.168.1.3}"
LIDAR_IFACE="${LIDAR_IFACE:-}"

if command -v ip >/dev/null 2>&1; then
  if [[ -n "${LIDAR_IFACE}" ]]; then
    ip_addr_output="$(ip -4 addr show dev "${LIDAR_IFACE}" 2>/dev/null || true)"
  else
    ip_addr_output="$(ip -4 addr show 2>/dev/null || true)"
  fi

  if ! grep -q "${LIDAR_HOST_IP}/" <<<"${ip_addr_output}"; then
    if [[ -n "${LIDAR_IFACE}" ]]; then
      echo "[livox-mid360] WARN: ${LIDAR_IFACE} 未检测到 ${LIDAR_HOST_IP}/24；MID360 配置需要主机 IP 为 ${LIDAR_HOST_IP}" >&2
      echo "[livox-mid360] WARN: 如确认使用 ${LIDAR_IFACE}，先执行: sudo ip addr add ${LIDAR_HOST_IP}/24 dev ${LIDAR_IFACE}" >&2
    else
      echo "[livox-mid360] WARN: 未在任意网卡检测到 ${LIDAR_HOST_IP}/24；MID360 配置需要主机 IP 为 ${LIDAR_HOST_IP}" >&2
      echo "[livox-mid360] WARN: 可用 LIDAR_IFACE=<网卡名> 指定网卡，或先给雷达网卡配置该 IP" >&2
    fi
  fi
fi

tmp_config="/tmp/slam_nav_mid360_${LIDAR_HOST_IP//./_}_${LIDAR_IP//./_}.json"
python3 - "${tmp_config}" "${LIDAR_HOST_IP}" "${LIDAR_IP}" <<'PY'
import json
import sys
from pathlib import Path

out_path = Path(sys.argv[1])
host_ip = sys.argv[2]
lidar_ip = sys.argv[3]
src_path = Path('src/livox_ros_driver2/config/MID360_config.json')

data = json.loads(src_path.read_text(encoding='utf-8'))
host = data['MID360']['host_net_info']
for key in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
    host[key] = host_ip
data['lidar_configs'][0]['ip'] = lidar_ip
out_path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
PY

echo "[livox-mid360] using host ${LIDAR_HOST_IP}, lidar ${LIDAR_IP}, config ${tmp_config}"
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py \
  user_config_path:="${tmp_config}" \
  multi_topic:=0 \
  "$@"
