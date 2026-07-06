#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

mode="${1:-auto}"
sample_seconds="${TASK1_RUNTIME_SAMPLE_SECONDS:-6}"
errors=0
warnings=0

ok() {
  echo "[task1-runtime] OK: $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[task1-runtime] WARN: $*" >&2
}

fail() {
  errors=$((errors + 1))
  echo "[task1-runtime] FAIL: $*" >&2
}

topic_info() {
  local topic="$1"
  ros2 topic info "${topic}" 2>/dev/null || true
}

publisher_count() {
  topic_info "$1" | awk -F': ' '/Publisher count/ {print $2; exit}'
}

subscription_count() {
  topic_info "$1" | awk -F': ' '/Subscription count/ {print $2; exit}'
}

require_topic_pub() {
  local topic="$1"
  local desc="$2"
  local count
  count="$(publisher_count "${topic}")"
  if [[ "${count:-0}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
    ok "${desc}: ${topic} publisher=${count}"
  else
    fail "${desc} 没有发布者: ${topic}"
  fi
}

require_topic_sub() {
  local topic="$1"
  local desc="$2"
  local count
  count="$(subscription_count "${topic}")"
  if [[ "${count:-0}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
    ok "${desc}: ${topic} subscriber=${count}"
  else
    fail "${desc} 没有订阅者: ${topic}"
  fi
}

warn_topic_pub() {
  local topic="$1"
  local desc="$2"
  local count
  count="$(publisher_count "${topic}")"
  if [[ "${count:-0}" =~ ^[0-9]+$ ]] && (( count > 0 )); then
    ok "${desc}: ${topic} publisher=${count}"
  else
    warn "${desc} 暂未发布: ${topic}"
  fi
}

sample_topic_hz() {
  local topic="$1"
  local desc="$2"
  local output
  output="$(timeout "${sample_seconds}" ros2 topic hz "${topic}" 2>&1 || true)"
  if grep -Eq 'average rate|avg rate|min:' <<<"${output}"; then
    local rate
    rate="$(grep -E 'average rate|avg rate' <<<"${output}" | tail -n 1 | sed 's/^/[task1-runtime]   /')"
    ok "${desc} 有连续消息: ${topic}"
    [[ -n "${rate}" ]] && echo "${rate}"
  else
    warn "${desc} 在 ${sample_seconds}s 内未测到连续消息: ${topic}"
  fi
}

check_tf() {
  local target="$1"
  local source="$2"
  local desc="$3"
  local output
  output="$(timeout 5 ros2 run tf2_ros tf2_echo "${target}" "${source}" 2>&1 || true)"
  if grep -Eq 'Translation:|At time|transform' <<<"${output}" && ! grep -Eq 'Invalid frame|Could not transform|Exception' <<<"${output}"; then
    ok "${desc}: ${target} -> ${source}"
  else
    fail "${desc} 不可用: ${target} -> ${source}"
    echo "${output}" | sed -n '1,8p' | sed 's/^/[task1-runtime]   /' >&2
  fi
}

check_lifecycle_active() {
  local node="$1"
  local output
  output="$(ros2 lifecycle get "${node}" 2>&1 || true)"
  if grep -Eq '^active([[:space:]]|$)' <<<"${output}"; then
    ok "Nav2 lifecycle active: ${node}"
  else
    fail "Nav2 lifecycle 未 active: ${node} (${output})"
  fi
}

print_usage() {
  cat <<'EOF'
用法:
  ./run.sh task1-runtime-check [mapping|nav|dynamic|auto]

说明:
  mapping  检查仿真 + 建图链路是否具备保存地图条件
  nav      检查仿真 + 已保存地图导航链路是否具备发目标点条件
  dynamic  检查动态障碍物/3D 增强演示链路的关键话题
  auto     根据当前 ROS 图自动判断，默认值

环境变量:
  TASK1_RUNTIME_SAMPLE_SECONDS=6  设置 topic hz 采样秒数
EOF
}

if [[ "${mode}" == "-h" || "${mode}" == "--help" ]]; then
  print_usage
  exit 0
fi

if [[ ! "${mode}" =~ ^(mapping|nav|dynamic|auto)$ ]]; then
  fail "未知模式: ${mode}"
  print_usage
  exit 2
fi

echo "[task1-runtime] 工作区: ${WORKSPACE_DIR}"
echo "[task1-runtime] 模式: ${mode}"

if ! command -v ros2 >/dev/null 2>&1; then
  fail "找不到 ros2 命令，请确认 ROS2 Humble 已安装"
  exit 1
fi

node_list="$(ros2 node list 2>/dev/null || true)"
topic_list="$(ros2 topic list 2>/dev/null || true)"
action_list="$(ros2 action list 2>/dev/null || true)"

if [[ -z "${node_list}" && -z "${topic_list}" ]]; then
  fail "当前没有发现 ROS2 节点或话题；请先启动 ./run.sh sim-static 与 mapping/nav"
  exit 1
fi

if [[ "${mode}" == "auto" ]]; then
  if grep -q '/bt_navigator' <<<"${node_list}" || grep -q '/navigate_to_pose' <<<"${action_list}"; then
    mode="nav"
  elif grep -q '/async_slam_toolbox_node' <<<"${node_list}" || grep -q '/map' <<<"${topic_list}"; then
    mode="mapping"
  else
    mode="mapping"
  fi
  echo "[task1-runtime] 自动识别为: ${mode}"
fi

echo
echo "[task1-runtime] 1/4 基础仿真与传感器"
require_topic_pub "/clock" "Gazebo 仿真时钟"
require_topic_pub "/livox/lidar" "Livox 点云"
warn_topic_pub "/livox/imu" "Livox IMU"
require_topic_pub "/Odometry" "FAST-LIO 里程计"
require_topic_pub "/scan" "2D LaserScan 投影"
sample_topic_hz "/scan" "LaserScan"
sample_topic_hz "/Odometry" "Odometry"

echo
echo "[task1-runtime] 2/4 TF 连通性"
check_tf "odom" "base_footprint" "底盘里程计 TF"
if [[ "${mode}" == "nav" || "${mode}" == "dynamic" ]]; then
  check_tf "map" "base_footprint" "导航全局 TF"
fi

echo
echo "[task1-runtime] 3/4 地图与速度接口"
require_topic_pub "/map" "地图"
require_topic_sub "/cmd_vel" "底盘速度订阅"
if [[ "${mode}" == "mapping" ]]; then
  warn_topic_pub "/slam_toolbox/graph_visualization" "slam_toolbox 图优化可视化"
fi

echo
echo "[task1-runtime] 4/4 Nav2/扩展链路"
if [[ "${mode}" == "nav" || "${mode}" == "dynamic" ]]; then
  for node in /map_server /amcl /planner_server /controller_server /bt_navigator; do
    check_lifecycle_active "${node}"
  done
  if grep -qx '/navigate_to_pose' <<<"${action_list}"; then
    ok "Nav2 action 可用: /navigate_to_pose"
  else
    fail "Nav2 action 不可用: /navigate_to_pose"
  fi
fi

if [[ "${mode}" == "dynamic" ]]; then
  warn_topic_pub "/terrain_map" "3D 地形分析一阶段"
  warn_topic_pub "/terrain_map_ext" "3D 地形分析二阶段"
  warn_topic_pub "/visual_obstacles" "RGB-D 松耦合障碍物"
fi

echo
echo "[task1-runtime] 结果: errors=${errors}, warnings=${warnings}"
if (( errors > 0 )); then
  echo "[task1-runtime] 结论: 当前链路还不能继续验收截图；先处理 FAIL 项。" >&2
  exit 1
fi

if (( warnings > 0 )); then
  echo "[task1-runtime] 结论: 主链路可继续，但 WARN 项建议截图前确认。"
else
  echo "[task1-runtime] 结论: 当前主链路状态良好，可以继续下一步。"
fi
