#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

STRICT=false
SHOW_ROWS=false
SHOW_NEXT=false
RECORD_FILE="tasks/task1/EXPERIMENT_RECORD.md"
FIG_DIR="tasks/task1/report_latex/figures"

source "${SCRIPT_DIR}/task1_state.sh"

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-experiment-check [--strict] [--show-rows] [--next]

说明：
  检查 task1 的静态避障 10 次实验记录，不启动 Gazebo、RViz 或 Nav2。
  普通模式用于查看当前还缺哪些字段；--strict 用于最终打包前确认 10 次实验已填完且成功率 >= 80%。

可选参数：
  --strict     将未填字段、成功率不足、截图文件缺失等 warning 视为未通过。
  --show-rows 逐行输出 10 次静态避障实验的解析结果，方便核对表格。
  --next      只额外提示下一条最应该补的实验记录和推荐填写格式。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --strict)
      STRICT=true
      ;;
    --show-rows)
      SHOW_ROWS=true
      ;;
    --next)
      SHOW_NEXT=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-experiment] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

errors=0
warnings=0

ok() {
  echo "[task1-experiment] OK: $*"
}

warn() {
  warnings=$((warnings + 1))
  echo "[task1-experiment] WARN: $*" >&2
}

fail() {
  errors=$((errors + 1))
  echo "[task1-experiment] FAIL: $*" >&2
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

is_pending() {
  local value
  value="$(trim "$1")"
  [[ -z "${value}" || "${value}" == "-" || "${value}" == *"待填"* || "${value}" == *"待补"* || "${value}" == *"TODO"* || "${value}" == *"todo"* ]]
}

is_positive() {
  local value="$1"
  [[ "${value}" == *"是"* || "${value}" == *"成功"* || "${value}" == *"通过"* || "${value}" == *"到达"* || "${value}" == *"yes"* || "${value}" == *"Yes"* || "${value}" == *"true"* || "${value}" == "1" ]]
}

is_negative() {
  local value="$1"
  [[ "${value}" == *"否"* || "${value}" == *"失败"* || "${value}" == *"未"* || "${value}" == *"无"* || "${value}" == *"no"* || "${value}" == *"No"* || "${value}" == *"false"* || "${value}" == "0" ]]
}

check_screenshot_reference() {
  local trial="$1"
  local shot="$2"
  local found_any=false
  local missing_any=false

  while IFS= read -r image_name; do
    [[ -z "${image_name}" ]] && continue
    found_any=true
    local image_base
    image_base="$(basename "${image_name}")"
    if [[ -f "${image_name}" || -f "${FIG_DIR}/${image_base}" ]]; then
      :
    else
      missing_any=true
      warn "第 ${trial} 次实验记录引用了截图 ${image_name}，但未在当前路径或 ${FIG_DIR}/ 中找到"
    fi
  done < <(grep -oE '[[:alnum:]_./-]+\.png' <<<"${shot}" || true)

  if [[ "${found_any}" == "false" && "${shot}" == *".png"* ]]; then
    warn "第 ${trial} 次实验截图字段包含 .png，但无法解析出文件名: ${shot}"
  elif [[ "${missing_any}" == "false" ]]; then
    return 0
  fi
}

echo "[task1-experiment] 工作区: ${WORKSPACE_DIR}"
echo "[task1-experiment] 实验记录: ${RECORD_FILE}"

if [[ ! -f "${RECORD_FILE}" ]]; then
  fail "缺少实验记录文件: ${RECORD_FILE}"
  exit 1
fi

if task1_load_state; then
  task1_print_state | sed 's/^/[task1-experiment]   /'
  while IFS= read -r issue; do
    [[ -n "${issue}" ]] && warn "${issue}"
  done < <(task1_state_issues)
else
  fail "Task1 状态文件无效"
fi

for marker in "导航方案标识" "证据状态" "Git 基线" "实验日期" "导航配置"; do
  if grep -Fq "${marker}" "${RECORD_FILE}"; then
    ok "实验批次包含元数据: ${marker}"
  else
    warn "实验批次缺少元数据: ${marker}"
  fi
done

if [[ ! -d "${FIG_DIR}" ]]; then
  warn "截图目录不存在: ${FIG_DIR}"
fi

in_table=false
rows=0
completed_rows=0
success_count=0
failed_count=0
collision_count=0
unknown_success_count=0
first_missing_trial=""
first_missing_fields=""

while IFS= read -r line; do
  if [[ "${line}" == *"| 编号 | 起点 | 目标点 |"* && "${line}" == *"是否成功"* ]]; then
    in_table=true
    continue
  fi

  if [[ "${in_table}" != "true" ]]; then
    continue
  fi

  if [[ "${line}" != "|"* ]]; then
    if (( rows > 0 )); then
      break
    fi
    continue
  fi

  if [[ "${line}" == *"---"* ]]; then
    continue
  fi

  row="${line#|}"
  row="${row%|}"
  IFS='|' read -r c_num c_start c_goal c_obstacle c_arrived c_collision c_success c_shot c_note c_extra <<<"${row}"

  num="$(trim "${c_num:-}")"
  if [[ ! "${num}" =~ ^[0-9]+$ ]]; then
    continue
  fi

  start="$(trim "${c_start:-}")"
  goal="$(trim "${c_goal:-}")"
  obstacle="$(trim "${c_obstacle:-}")"
  arrived="$(trim "${c_arrived:-}")"
  collision="$(trim "${c_collision:-}")"
  success="$(trim "${c_success:-}")"
  shot="$(trim "${c_shot:-}")"
  note="$(trim "${c_note:-}")"

  rows=$((rows + 1))
  missing_fields=()
  is_pending "${start}" && missing_fields+=("起点")
  is_pending "${goal}" && missing_fields+=("目标点")
  is_pending "${obstacle}" && missing_fields+=("主要经过障碍")
  is_pending "${arrived}" && missing_fields+=("是否到达")
  is_pending "${collision}" && missing_fields+=("是否碰撞")
  is_pending "${success}" && missing_fields+=("是否成功")
  is_pending "${shot}" && missing_fields+=("截图编号")

  if (( ${#missing_fields[@]} == 0 )); then
    completed_rows=$((completed_rows + 1))
  else
    warn "第 ${num} 次静态避障实验未填完整: ${missing_fields[*]}"
    if [[ -z "${first_missing_trial}" ]]; then
      first_missing_trial="${num}"
      first_missing_fields="${missing_fields[*]}"
    fi
  fi

  if is_negative "${success}"; then
    failed_count=$((failed_count + 1))
  elif is_positive "${success}"; then
    success_count=$((success_count + 1))
  elif ! is_pending "${success}"; then
    unknown_success_count=$((unknown_success_count + 1))
    warn "第 ${num} 次实验“是否成功”字段无法判定，请写成 是/否 或 成功/失败: ${success}"
  fi

  if is_positive "${collision}"; then
    collision_count=$((collision_count + 1))
  fi

  if is_positive "${success}" && is_negative "${arrived}"; then
    warn "第 ${num} 次实验标记为成功，但“是否到达”为否，请核对"
  fi

  if is_positive "${success}" && is_positive "${collision}"; then
    warn "第 ${num} 次实验标记为成功，但“是否碰撞”为是，请核对成功判定"
  fi

  if ! is_pending "${shot}"; then
    check_screenshot_reference "${num}" "${shot}"
  fi

  if [[ "${SHOW_ROWS}" == "true" ]]; then
    printf '[task1-experiment] row %s: 起点=%s | 目标=%s | 到达=%s | 碰撞=%s | 成功=%s | 截图=%s | 备注=%s\n' \
      "${num}" "${start}" "${goal}" "${arrived}" "${collision}" "${success}" "${shot}" "${note}"
  fi
done < "${RECORD_FILE}"

if (( rows == 0 )); then
  fail "未找到静态避障 10 次实验表，请确认 ${RECORD_FILE} 中存在“编号/起点/目标点/是否成功”表格"
elif (( rows < 10 )); then
  warn "静态避障实验表只有 ${rows} 行，课程验收建议记录 10 次"
elif (( rows > 10 )); then
  warn "静态避障实验表解析到 ${rows} 行；如包含额外行，请确认是否仍按前 10 次统计"
else
  ok "静态避障实验表包含 10 行"
fi

if (( completed_rows == 10 )); then
  ok "10 次静态避障实验字段已填完整"
else
  warn "静态避障实验完整记录数: ${completed_rows}/10"
fi

if (( rows > 0 )); then
  rate=$((success_count * 100 / rows))
else
  rate=0
fi

echo "[task1-experiment] 统计: 成功=${success_count}, 失败=${failed_count}, 碰撞=${collision_count}, 成功率=${rate}%"

if (( unknown_success_count > 0 )); then
  warn "有 ${unknown_success_count} 次实验的成功字段无法自动判定"
fi

if (( rows >= 10 && success_count >= 8 )); then
  ok "静态避障成功率满足 >= 80% 的课程要求"
else
  warn "静态避障成功率尚未证明达到 >= 80%；10 次实验中至少需要 8 次成功"
fi

if [[ "${SHOW_NEXT}" == "true" ]]; then
  echo
  if [[ -n "${first_missing_trial}" ]]; then
    echo "[task1-experiment] 下一条建议补第 ${first_missing_trial} 次静态避障实验，当前缺字段: ${first_missing_fields}"
    echo "[task1-experiment] 推荐填写格式:"
    echo "| ${first_missing_trial} | 起点坐标或区域 | 目标点坐标或区域 | 经过的主要障碍物 | 是/否 | 是/否 | 是/否 | 图8-2/图8-3/图8-4 或补充 PNG | 失败原因或恢复过程 |"
    echo "[task1-experiment] 判定口径: 到达=停在目标附近；碰撞=碰到障碍物/墙/动态物体；成功=到达且无碰撞、无长期卡死。"
  else
    ok "10 次静态避障实验字段已经填完整；下一步运行 ./run.sh task1-experiment-check --strict"
  fi
fi

echo "[task1-experiment] 结构错误: ${errors}, 提交前提醒: ${warnings}"

if [[ "${errors}" -ne 0 ]]; then
  exit 1
fi

if [[ "${STRICT}" == "true" && "${warnings}" -ne 0 ]]; then
  echo "[task1-experiment] --strict 模式下 warning 也视为未完成。" >&2
  exit 1
fi

exit 0
