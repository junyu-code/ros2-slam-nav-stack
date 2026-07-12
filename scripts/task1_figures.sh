#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

FIG_DIR="tasks/task1/report_latex/figures"

required_figs=(
  "6-1|fig_6_1_gazebo_world.png|Gazebo 静态场地总览|./run.sh clean && ./run.sh sim-static"
  "7-1|fig_7_1_mapping_rviz.png|RViz 建图过程|./run.sh mapping + ./run.sh teleop"
  "7-2|fig_7_2_saved_map.png|保存后的地图|./run.sh save-map nav_test_map"
  "8-1|fig_8_1_nav2_map_loaded.png|Nav2 加载地图|./run.sh nav"
  "8-2|fig_8_2_global_path.png|全局路径|RViz 发送 2D Goal Pose"
  "8-3|fig_8_3_avoid_obstacle.png|静态避障过程|导航过程中截图"
  "8-4|fig_8_4_goal_reached.png|到达目标点|导航完成后截图"
  "9-1|fig_9_1_dynamic_obstacle.png|动态障碍物扩展示范|./run.sh sim-dynamic + ./run.sh nav-3d"
)

optional_figs=(
  "8-5|fig_8_5_backup_recovery.png|后退恢复过程|可选：拥挤障碍附近触发恢复行为"
  "9-2|fig_9_2_rgbd_visual_obstacles.png|RGB-D 近场障碍点云|可选：./run.sh sim-dynamic-rgbd + ./run.sh nav-full"
  "9-3|fig_9_3_perception_adapter.png|感知接口说明|可选：展示感知适配接口或话题"
  "9-4|fig_9_4_large_arena_overview.png|大场地鲁棒性扩展总览|可选：./run.sh large-arena-collision"
  "9-5|fig_9_5_large_arena_relocalization.png|大场地重定位扩展验证|可选：./run.sh large-arena-nav"
  "9-6|fig_9_6_relocalization_log.png|AMCL/GICP 重定位诊断日志|可选：展示 /relocalization/status 或 AMCL 收敛状态"
  "9-7|fig_9_7_collision_recovery.png|碰撞扰动后恢复对比|可选：./run.sh teleop-manual-car"
  "10-1|fig_10_1_real_livox_network.png|实机 MID360 网络与驱动|可选：实机迁移材料"
  "10-2|fig_10_2_real_d435i_depth.png|实机 D435i RGB-D 数据|可选：实机迁移材料"
  "10-3|fig_10_3_real_fast_lio_mapping.png|实机 FAST-LIO 点云建图|可选：实机迁移材料"
  "10-4|fig_10_4_real_cmd_bridge_test.png|实机底盘安全桥测试|可选：实机迁移材料"
  "10-5|fig_10_5_real_runtime_diagnosis.png|实机运行诊断结果|可选：实机迁移材料"
)

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-figures
  ./run.sh task1-figures list
  ./run.sh task1-figures path <编号或文件名>
  ./run.sh task1-figures import <编号或文件名> <源图片> [--force]

说明：
  管理 task1 报告截图文件名，不启动 Gazebo、RViz、Nav2 或任何 ROS GUI。
  默认 list 只检查缺图；import 只复制真实截图，不生成或伪造实验图片。

编号示例：
  6-1、图6-1、fig_6_1、fig_6_1_gazebo_world.png

导入示例：
  ./run.sh task1-figures import 6-1 /mnt/c/Users/32149/Pictures/gazebo.png
  ./run.sh task1-figures import fig_8_2_global_path.png /tmp/path.png --force
EOF
}

all_fig_rows() {
  printf '%s\n' "${required_figs[@]}"
  printf '%s\n' "${optional_figs[@]}"
}

normalize_key() {
  local key="$1"
  key="${key,,}"
  key="${key%.png}"
  key="${key#图}"
  key="${key//_/-}"
  key="${key// /}"
  key="${key#fig-}"
  key="${key#fig}"
  key="${key#figure-}"
  key="${key#figure}"
  key="${key//--/-}"
  printf '%s' "${key}"
}

find_figure_row() {
  local query="$1"
  local normalized
  normalized="$(normalize_key "${query}")"

  while IFS='|' read -r id file desc hint; do
    local file_no_ext="${file%.png}"
    local file_key
    file_key="$(normalize_key "${file_no_ext}")"
    if [[ "${query}" == "${file}" || "${query}" == "${file_no_ext}" || "${normalized}" == "${id}" || "${normalized}" == "${file_key}" || "${file_key}" == "${normalized}-"* ]]; then
      printf '%s|%s|%s|%s\n' "${id}" "${file}" "${desc}" "${hint}"
      return 0
    fi
  done < <(all_fig_rows)

  return 1
}

is_png() {
  local path="$1"
  if command -v file >/dev/null 2>&1; then
    local info
    info="$(file -b "${path}" 2>/dev/null || true)"
    [[ "${info}" == PNG* ]]
  else
    [[ "$(head -c 8 "${path}" | od -An -tx1 | tr -d ' \n')" == "89504e470d0a1a0a" ]]
  fi
}

file_info() {
  local path="$1"
  if [[ ! -f "${path}" ]]; then
    printf '缺失'
    return
  fi
  printf '已存在，%s，更新时间 %s' \
    "$(stat -c '%s bytes' "${path}")" \
    "$(date -d "@$(stat -c '%Y' "${path}")" '+%Y-%m-%d %H:%M:%S')"
}

resolve_source_path() {
  local source="$1"
  if [[ -f "${source}" ]]; then
    printf '%s' "${source}"
    return 0
  fi

  if command -v wslpath >/dev/null 2>&1 && [[ "${source}" =~ ^[A-Za-z]:\\ ]]; then
    local converted
    converted="$(wslpath -u "${source}" 2>/dev/null || true)"
    if [[ -n "${converted}" && -f "${converted}" ]]; then
      printf '%s' "${converted}"
      return 0
    fi
  fi

  return 1
}

list_figures() {
  local missing=0
  local required_total=0
  local required_done=0

  mkdir -p "${FIG_DIR}"

  echo "[task1-figures] 截图目录: ${FIG_DIR}"
  echo
  echo "必需截图："
  printf '| 编号 | 文件名 | 说明 | 状态 | 建议动作 |\n'
  printf '|---|---|---|---|---|\n'
  for item in "${required_figs[@]}"; do
    IFS='|' read -r id file desc hint <<<"${item}"
    required_total=$((required_total + 1))
    if [[ -f "${FIG_DIR}/${file}" ]]; then
      required_done=$((required_done + 1))
    else
      missing=$((missing + 1))
    fi
    printf '| %s | `%s` | %s | %s | `%s` |\n' "${id}" "${file}" "${desc}" "$(file_info "${FIG_DIR}/${file}")" "${hint}"
  done

  echo
  echo "可选截图："
  printf '| 编号 | 文件名 | 说明 | 状态 |\n'
  printf '|---|---|---|---|\n'
  for item in "${optional_figs[@]}"; do
    IFS='|' read -r id file desc _hint <<<"${item}"
    printf '| %s | `%s` | %s | %s |\n' "${id}" "${file}" "${desc}" "$(file_info "${FIG_DIR}/${file}")"
  done

  echo
  echo "[task1-figures] 必需截图完成: ${required_done}/${required_total}"
  if (( missing > 0 )); then
    echo "[task1-figures] 还有 ${missing} 张必需截图未补。"
  else
    echo "[task1-figures] 必需截图已补齐，建议运行 ./run.sh task1-report-audit。"
  fi
}

print_path() {
  local query="$1"
  local row
  if ! row="$(find_figure_row "${query}")"; then
    echo "[task1-figures] 未找到截图编号或文件名: ${query}" >&2
    exit 2
  fi
  IFS='|' read -r _id file _desc _hint <<<"${row}"
  printf '%s/%s\n' "${FIG_DIR}" "${file}"
}

import_figure() {
  local query="${1:-}"
  local source="${2:-}"
  local force=false

  shift 2 || true
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --force)
        force=true
        ;;
      -h|--help|help)
        usage
        exit 0
        ;;
      *)
        echo "[task1-figures] 未知参数: $1" >&2
        usage >&2
        exit 2
        ;;
    esac
    shift
  done

  if [[ -z "${query}" || -z "${source}" ]]; then
    echo "[task1-figures] import 需要 <编号或文件名> 和 <源图片>" >&2
    usage >&2
    exit 2
  fi

  local row
  if ! row="$(find_figure_row "${query}")"; then
    echo "[task1-figures] 未找到截图编号或文件名: ${query}" >&2
    exit 2
  fi

  local source_path
  if ! source_path="$(resolve_source_path "${source}")"; then
    echo "[task1-figures] 源图片不存在: ${source}" >&2
    echo "[task1-figures] Windows 路径可写成 /mnt/c/Users/...，或直接传 C:\\... 路径。" >&2
    exit 1
  fi

  if ! is_png "${source_path}"; then
    echo "[task1-figures] 源文件不是 PNG 图片，未导入: ${source_path}" >&2
    echo "[task1-figures] 请先用截图工具另存为 PNG，或转换后再导入。" >&2
    exit 1
  fi

  IFS='|' read -r id file desc _hint <<<"${row}"
  mkdir -p "${FIG_DIR}"
  local target="${FIG_DIR}/${file}"
  if [[ -f "${target}" && "${force}" != "true" ]]; then
    echo "[task1-figures] 目标已存在，未覆盖: ${target}" >&2
    echo "[task1-figures] 如确认要替换，追加 --force。" >&2
    exit 1
  fi

  cp "${source_path}" "${target}"
  echo "[task1-figures] 已导入 ${id} ${desc}"
  echo "[task1-figures] ${source_path} -> ${target}"
  echo "[task1-figures] 建议继续运行: ./run.sh task1-report-audit"
}

command="${1:-list}"
case "${command}" in
  list|status|"")
    list_figures
    ;;
  path)
    if [[ -z "${2:-}" ]]; then
      echo "[task1-figures] path 需要 <编号或文件名>" >&2
      usage >&2
      exit 2
    fi
    print_path "$2"
    ;;
  import)
    shift
    import_figure "$@"
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "[task1-figures] 未知命令: ${command}" >&2
    usage >&2
    exit 2
    ;;
esac
