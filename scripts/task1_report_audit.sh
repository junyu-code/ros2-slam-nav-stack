#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

STRICT=false
REPORT_TEX="tasks/task1/report_latex/main.tex"
REPORT_PDF="tasks/task1/report_latex/main.pdf"
FIG_DIR="tasks/task1/report_latex/figures"
EXPERIMENT_RECORD="tasks/task1/EXPERIMENT_RECORD.md"
GENERATED_TRIALS_TEX="tasks/task1/report_latex/generated_static_trials.tex"
EVIDENCE_MANIFEST="tasks/task1/evidence_media/MANIFEST.tsv"
EVIDENCE_SHA256="tasks/task1/evidence_media/SHA256SUMS"
EVIDENCE_VIDEO="tasks/task1/evidence_media/task1_rgbd_dynamic_demo_720p.mp4"

source "${SCRIPT_DIR}/task1_state.sh"

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-report-audit [--strict]

说明：
  专门审计 task1 结课报告材料，不启动 Gazebo、RViz、Nav2 或任何 ROS GUI。
  普通模式用于找缺图、待填字段、PDF 是否过期；--strict 用于最终提交前把 warning 也视为未完成。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --strict)
      STRICT=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-report] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

errors=0
warnings=0

ok() {
  echo "[task1-report] OK: $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[task1-report] WARN: $*" >&2
}

fail() {
  errors=$((errors + 1))
  echo "[task1-report] FAIL: $*" >&2
}

check_file() {
  local path="$1"
  local desc="$2"
  if [[ -f "${path}" ]]; then
    ok "${desc}: ${path}"
  else
    fail "缺少 ${desc}: ${path}"
  fi
}

check_png_image() {
  local path="$1"
  local desc="$2"

  if [[ ! -f "${path}" ]]; then
    return 0
  fi

  local size
  size="$(stat -c '%s' "${path}" 2>/dev/null || echo 0)"
  if (( size < 1024 )); then
    warn "${desc} 文件过小，可能不是有效截图: ${path}"
    return 0
  fi

  if command -v file >/dev/null 2>&1; then
    local file_info
    file_info="$(file -b "${path}" 2>/dev/null || true)"
    if [[ "${file_info}" != PNG* ]]; then
      warn "${desc} 不是 PNG 图片或文件头异常: ${path} (${file_info})"
    else
      ok "${desc} PNG 格式有效: ${path}"
    fi
  fi
}

contains_text() {
  local file="$1"
  local text="$2"
  grep -Fq "${text}" "${file}" 2>/dev/null
}

count_matches() {
  local pattern="$1"
  shift
  { grep -RInE "${pattern}" "$@" 2>/dev/null || true; } | wc -l | tr -d ' '
}

echo "[task1-report] 工作区: ${WORKSPACE_DIR}"

if task1_load_state; then
  task1_print_state | sed 's/^/[task1-report]   /'
  while IFS= read -r issue; do
    [[ -n "${issue}" ]] && warn "${issue}"
  done < <(task1_state_issues)
else
  fail "Task1 状态文件无效"
fi

check_file "${REPORT_TEX}" "LaTeX 结课报告源文件"
check_file "${GENERATED_TRIALS_TEX}" "LaTeX 静态避障实验表生成片段"
check_file "${EVIDENCE_MANIFEST}" "媒体证据说明"
check_file "${EVIDENCE_SHA256}" "媒体证据校验和"
check_file "${EVIDENCE_VIDEO}" "RGB-D 与动态避障连续演示视频"

if [[ -f "${EVIDENCE_VIDEO}" ]]; then
  video_size="$(stat -c '%s' "${EVIDENCE_VIDEO}" 2>/dev/null || echo 0)"
  if (( video_size >= 1048576 )); then
    ok "连续演示视频大小合理: ${video_size} bytes"
  else
    warn "连续演示视频过小，可能不是有效附件: ${EVIDENCE_VIDEO}"
  fi
fi

if [[ -f "${REPORT_TEX}" ]]; then
  for item in \
    "佘俊谕|学生姓名" \
    "3232072072234|学号" \
    "机器人工程23-1班|班级" \
    "梁勇|指导教师"; do
    IFS='|' read -r text desc <<<"${item}"
    if contains_text "${REPORT_TEX}" "${text}"; then
      ok "LaTeX 报告包含${desc}: ${text}"
    else
      fail "LaTeX 报告缺少${desc}: ${text}"
    fi
  done
fi

required_figs=(
  "fig_6_1_gazebo_world.png|Gazebo 静态场地总览"
  "fig_7_1_mapping_rviz.png|RViz 建图过程"
  "fig_7_2_saved_map.png|保存后的地图"
  "fig_8_1_nav2_map_loaded.png|Nav2 加载地图"
  "fig_8_2_global_path.png|全局路径"
  "fig_8_3_avoid_obstacle.png|静态避障过程"
  "fig_8_4_goal_reached.png|到达目标点"
  "fig_9_1_dynamic_obstacle.png|动态障碍物扩展示范"
)

if [[ -d "${FIG_DIR}" ]]; then
  ok "截图目录存在: ${FIG_DIR}"
else
  warn "截图目录不存在: ${FIG_DIR}"
fi

missing_figs=0
for item in "${required_figs[@]}"; do
  IFS='|' read -r file desc <<<"${item}"
  if [[ -f "${REPORT_TEX}" ]] && contains_text "${REPORT_TEX}" "${file}"; then
    ok "LaTeX 报告已引用 ${desc}: ${file}"
  else
    fail "LaTeX 报告未引用必需截图 ${desc}: ${file}"
  fi

  if [[ -f "${FIG_DIR}/${file}" ]]; then
    ok "必需截图已补: ${file}"
    check_png_image "${FIG_DIR}/${file}" "必需截图 ${file}"
  else
    missing_figs=$((missing_figs + 1))
    warn "必需截图未补: ${FIG_DIR}/${file}"
  fi
done

optional_figs=(
  "fig_7_3_pcd_map.png|PCD 点云地图预览"
  "fig_7_4_mapping_pose_failure.png|建图位姿失败证据"
  "fig_8_5_backup_recovery.png|后退恢复过程"
  "fig_8_6_initial_navigation_failure.png|初版导航失败证据"
  "fig_8_7_amcl_navigation_improved.png|AMCL 导航改进证据"
  "fig_9_2_rgbd_visual_obstacles.png|RGB-D 近场障碍点云"
  "fig_9_4_rgbd_dynamic_video_frames.png|RGB-D 动态演示视频四时刻画面"
  "fig_9_3_perception_adapter.png|感知接口说明"
  "fig_9_4_large_arena_overview.png|大场地鲁棒性扩展总览"
  "fig_9_5_large_arena_relocalization.png|大场地重定位扩展验证"
  "fig_9_6_relocalization_log.png|AMCL/GICP 重定位诊断日志"
  "fig_9_7_collision_recovery.png|碰撞扰动后恢复对比"
  "fig_10_1_real_livox_network.png|实机 MID360 网络与驱动"
  "fig_10_2_real_d435i_depth.png|实机 D435i RGB-D 数据"
  "fig_10_3_real_fast_lio_mapping.png|实机 FAST-LIO 点云建图"
  "fig_10_4_real_cmd_bridge_test.png|实机底盘安全桥测试"
  "fig_10_5_real_runtime_diagnosis.png|实机运行诊断结果"
)

for item in "${optional_figs[@]}"; do
  IFS='|' read -r file desc <<<"${item}"
  if [[ -f "${FIG_DIR}/${file}" ]]; then
    ok "可选截图已补: ${desc}: ${file}"
    check_png_image "${FIG_DIR}/${file}" "可选截图 ${file}"
  fi
done

placeholder_count="$(count_matches '待填|待补|待替换|【待插图' "${REPORT_TEX}" "${GENERATED_TRIALS_TEX}")"
if [[ "${placeholder_count}" == "0" ]]; then
  ok "报告源文件中未发现待填/待替换占位"
else
  warn "报告源文件仍有 ${placeholder_count} 处待填/待替换占位"
fi

competition_hits="$(grep -RInE 'RoboMaster|RMUC|哨兵|云台|比赛模式|比赛业务' "${REPORT_TEX}" 2>/dev/null || true)"
if [[ -z "${competition_hits}" ]]; then
  ok "报告中未发现明显无关比赛业务字段"
else
  warn "报告中可能仍有无关比赛业务字段:"
  echo "${competition_hits}" >&2
fi

if [[ -f "${REPORT_PDF}" ]]; then
  ok "结课报告 PDF 存在: ${REPORT_PDF}"
  if [[ "${REPORT_TEX}" -nt "${REPORT_PDF}" ]]; then
    warn "LaTeX 源文件比 PDF 新，建议重新运行 ./run.sh task1-build-report"
  fi
  if [[ -f "${GENERATED_TRIALS_TEX}" && "${GENERATED_TRIALS_TEX}" -nt "${REPORT_PDF}" ]]; then
    warn "静态避障实验表生成片段比 PDF 新，建议重新运行 ./run.sh task1-build-report"
  fi
  if [[ -d "${FIG_DIR}" ]]; then
    newest_fig="$(find "${FIG_DIR}" -type f -name '*.png' -newer "${REPORT_PDF}" -print -quit 2>/dev/null || true)"
    if [[ -n "${newest_fig}" ]]; then
      warn "存在比 PDF 更新的截图 ${newest_fig}，建议重新运行 ./run.sh task1-build-report"
    fi
  fi
else
  warn "结课报告 PDF 不存在，补齐材料后运行 ./run.sh task1-build-report"
fi

if [[ -f "${EXPERIMENT_RECORD}" && -f "${GENERATED_TRIALS_TEX}" && "${EXPERIMENT_RECORD}" -nt "${GENERATED_TRIALS_TEX}" ]]; then
  warn "实验记录比生成表格新，建议运行 ./run.sh task1-sync-report"
fi

echo "[task1-report] 缺失必需截图: ${missing_figs}/${#required_figs[@]}"
echo "[task1-report] 结构错误: ${errors}, 提交前提醒: ${warnings}"

if [[ "${errors}" -ne 0 ]]; then
  exit 1
fi

if [[ "${STRICT}" == "true" && "${warnings}" -ne 0 ]]; then
  echo "[task1-report] --strict 模式下 warning 也视为未完成。" >&2
  exit 1
fi

exit 0
