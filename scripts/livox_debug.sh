#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

lidar_ip="${LIDAR_IP:-192.168.1.3}"
host_ip="${LIDAR_HOST_IP:-192.168.1.50}"
alt_lidar_ips="${LIDAR_IP_CANDIDATES:-192.168.1.3 192.168.1.143}"

echo "[livox-debug] host expected: ${host_ip}"
echo "[livox-debug] lidar expected: ${lidar_ip}"

echo
echo "[livox-debug] IPv4 addresses:"
ip -4 addr show 2>/dev/null | sed -n '/^[0-9]/p;/inet /p' || true

echo
echo "[livox-debug] route to lidar:"
timeout 2s ip route get "${lidar_ip}" 2>/dev/null || true

echo
echo "[livox-debug] ping lidar candidates:"
for ip_addr in ${alt_lidar_ips}; do
  echo "--- ${ip_addr}"
  timeout 2s ping -c 1 -W 1 "${ip_addr}" || true
done

echo
echo "[livox-debug] neighbor table:"
timeout 2s ip neigh show 2>/dev/null | sed -n '/192\.168\.1\./p' || true

echo
echo "[livox-debug] ROS nodes:"
timeout 2s ros2 node list 2>/dev/null | sed -n '/livox/p;/slam/p;/filter/p' || true

echo
echo "[livox-debug] /livox topics:"
timeout 2s ros2 topic list 2>/dev/null | sed -n '/^\/livox/p' || true

echo
echo "[livox-debug] expected topic publishers:"
for topic in "/livox/lidar" "/livox/imu" "/imu/data"; do
  echo "--- ${topic}"
  timeout 1s ros2 topic info "${topic}" -v 2>/dev/null || true
done

echo
echo "[livox-debug] livox node params:"
timeout 2s ros2 param dump /livox_lidar_publisher 2>/dev/null || true

echo
echo "[livox-debug] duplicate livox processes:"
pgrep -af "livox_ros_driver2_node|ros2 launch livox_ros_driver2" || true

echo
echo "[livox-debug] firewall:"
timeout 2s sudo ufw status 2>/dev/null || timeout 2s ufw status 2>/dev/null || true

echo
echo "[livox-debug] UDP sockets on Livox ports:"
timeout 2s ss -lunp 2>/dev/null | sed -n '/:5610/p;/:5620/p;/:5630/p;/:5640/p;/:5650/p' || true

echo
echo "[livox-debug] verdict:"
livox_processes="$(pgrep -af "livox_ros_driver2_node|ros2 launch livox_ros_driver2" || true)"
route_iface="$(ip route get "${lidar_ip}" 2>/dev/null | awk '/ dev / {for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' || true)"
neigh_iface="$(ip neigh show "${lidar_ip}" 2>/dev/null | awk '/lladdr/ {for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' || true)"
if [[ -n "${route_iface}" && -n "${neigh_iface}" && "${route_iface}" != "${neigh_iface}" ]]; then
  echo "- Route mismatch: ${lidar_ip} routes via ${route_iface}, but ARP saw it on ${neigh_iface}."
  echo "- real-mapping will try to add a /32 route via ${neigh_iface}; sudo password may be required."
fi
if ! timeout 2s ros2 topic list 2>/dev/null | grep -Eq '^/livox/(lidar|imu)'; then
  if [[ -n "${livox_processes}" ]]; then
    echo "- No /livox topics. The driver process is running but has not discovered/received MID360 data."
    echo "- Check MID360 IP/power/cable/firewall. This is before FAST-LIO, slam_toolbox, RViz, or depth camera."
  else
    echo "- No /livox topics and no Livox driver process is running."
    echo "- Start via scripts/start_livox_mid360.sh, then run mapping or navigation."
  fi
fi
