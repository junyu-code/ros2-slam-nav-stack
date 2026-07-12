#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

mode="auto"
SAVE_SUMMARY=false
SUMMARY_PATH="artifacts/task1/runtime/latest.txt"
HISTORY_DIR="artifacts/task1/runtime/history"
SAVE_HISTORY=true
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

print_record_guidance() {
  echo
  echo "[task1-runtime] 记录填写建议"
  case "${mode}" in
    mapping)
      cat <<'EOF'
- 若上方 FAIL=0，可在 `tasks/task1/EXPERIMENT_RECORD.md` 的“2.2 运行检查”中把已通过项填写为“通过”。
- `/livox/lidar`、`/livox/imu`、`/Odometry`、`/cloud_registered`、`/scan` 和 `/map` 分别对应 LiDAR、IMU、里程计、注册点云、二维激光和地图输出。
- 使用 `--save` 保存的快照可作为文字证据；RViz/Gazebo 截图仍需另存为图 7-1 和图 7-2。
EOF
      ;;
    nav)
      cat <<'EOF'
- 若上方 FAIL=0，可在 `tasks/task1/EXPERIMENT_RECORD.md` 的“3.2 导航检查”中把地图、里程计、激光、TF、速度指令和 Nav2 状态填写为“通过”。
- 若 `map -> base_footprint` 可用且 `/bt_navigator` 为 active，下一步可以在 RViz 中发送目标点并采集图 8-1 至图 8-4。
- 若出现 TF 或 lifecycle FAIL，先不要截图验收；优先重新执行 `./run.sh clean && ./run.sh sim-static`，再另开终端运行 `./run.sh nav`。
EOF
      ;;
    dynamic)
      cat <<'EOF'
- 动态障碍物只作为扩展示范，不计入 10 次静态避障成功率。
- 若 `/terrain_map`、`/terrain_map_ext` 或 `/visual_obstacles` 只有 WARN，可先截图记录为“扩展链路仍需调参”，不要写成稳定闭环能力。
- 图 9-1 应同时展示动态障碍物、机器人/路径和局部代价地图或终端运行状态。
EOF
      ;;
  esac
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
  ./run.sh task1-runtime-check [mapping|nav|dynamic|auto] [--save] [--output <path>] [--no-history]

说明:
  mapping  检查仿真 + 建图链路是否具备保存地图条件
  nav      检查仿真 + 已保存地图导航链路是否具备发目标点条件
  dynamic  检查动态障碍物/3D 增强演示链路的关键话题
  auto     根据当前 ROS 图自动判断，默认值
  --save       将本次检查终端输出保存为本地文本，默认写入 artifacts/task1/runtime/latest.txt
               同时归档到 artifacts/task1/runtime/history/<时间>_<模式>.txt。
  --output     配合 --save 指定 latest 快照路径
  --no-history 只写 latest 快照，不写入历史目录

环境变量:
  TASK1_RUNTIME_SAMPLE_SECONDS=6  设置 topic hz 采样秒数
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    mapping|nav|dynamic|auto)
      mode="$1"
      ;;
    --save)
      SAVE_SUMMARY=true
      ;;
    --output)
      if [[ -z "${2:-}" ]]; then
        echo "[task1-runtime] --output 需要路径参数" >&2
        exit 2
      fi
      SUMMARY_PATH="$2"
      shift
      ;;
    --no-history)
      SAVE_HISTORY=false
      ;;
    -h|--help|help)
      print_usage
      exit 0
      ;;
    *)
      fail "未知参数: $1"
      print_usage
      exit 2
      ;;
  esac
  shift
done

if [[ ! "${mode}" =~ ^(mapping|nav|dynamic|auto)$ ]]; then
  fail "未知模式: ${mode}"
  print_usage
  exit 2
fi

if [[ "${SAVE_SUMMARY}" == "true" && "${TASK1_RUNTIME_SAVE_ACTIVE:-false}" != "true" ]]; then
  tmp_output="$(mktemp)"
  tmp_summary="$(mktemp)"
  cleanup_runtime_summary() {
    rm -f "${tmp_output}" "${tmp_summary}"
  }
  trap cleanup_runtime_summary EXIT

  set +e
  TASK1_RUNTIME_SAVE_ACTIVE=true "$0" "${mode}" 2>&1 | tee "${tmp_output}"
  status="${PIPESTATUS[0]}"
  set -e

  generated_at="$(date '+%Y-%m-%d %H:%M:%S %z')"
  timestamp="$(date '+%Y%m%d_%H%M%S')"
  history_path="${HISTORY_DIR}/${timestamp}_${mode}.txt"

  mkdir -p "$(dirname "${SUMMARY_PATH}")"
  {
    echo "Task1 运行时检查快照"
    echo "自动生成时间：${generated_at}"
    echo "生成命令：./run.sh task1-runtime-check ${mode} --save"
    echo "说明：本文件记录真实运行中的 ROS 图检查结果，可作为填写 EXPERIMENT_RECORD.md 的依据，但不能替代截图。"
    echo
    cat "${tmp_output}"
  } > "${tmp_summary}"

  cp "${tmp_summary}" "${SUMMARY_PATH}"
  echo "[task1-runtime] 已写入运行时快照: ${SUMMARY_PATH}"
  if [[ "${SAVE_HISTORY}" == "true" ]]; then
    mkdir -p "${HISTORY_DIR}"
    cp "${tmp_summary}" "${history_path}"
    echo "[task1-runtime] 已归档运行时快照: ${history_path}"
  fi
  exit "${status}"
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
require_topic_pub "/cloud_registered" "FAST-LIO 注册点云"
require_topic_pub "/scan" "2D LaserScan 投影"
sample_topic_hz "/scan" "LaserScan"
sample_topic_hz "/Odometry" "Odometry"
sample_topic_hz "/cloud_registered" "注册点云"

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
  warn_topic_pub "/nav_camera/color/image_raw" "RGB-D 相机 RGB 图像"
  warn_topic_pub "/nav_camera/depth/image_raw" "RGB-D 相机深度图像"
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

print_record_guidance
