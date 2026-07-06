#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

RECORD_FILE="tasks/task1/EXPERIMENT_RECORD.md"
GENERATED_MD="tasks/task1/STATIC_TRIALS_TABLE.md"
GENERATED_TEX="tasks/task1/report_latex/generated_static_trials.tex"
QUIET=false

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-sync-report [--quiet]

说明：
  从 tasks/task1/EXPERIMENT_RECORD.md 中读取 10 次静态避障实验表，
  生成报告侧可复用的 Markdown 表格和 LaTeX 表格片段。
  该命令不启动 Gazebo、RViz、Nav2，也不会编造实验结果；未填写的字段会继续显示为“待填”。

生成文件：
  tasks/task1/STATIC_TRIALS_TABLE.md
  tasks/task1/report_latex/generated_static_trials.tex
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --quiet)
      QUIET=true
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-sync] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

log() {
  if [[ "${QUIET}" != "true" ]]; then
    echo "[task1-sync] $*"
  fi
}

trim() {
  local value="$1"
  value="${value#"${value%%[![:space:]]*}"}"
  value="${value%"${value##*[![:space:]]}"}"
  printf '%s' "${value}"
}

normalize_cell() {
  local value
  value="$(trim "$1")"
  if [[ -z "${value}" ]]; then
    value="待填"
  fi
  printf '%s' "${value}"
}

latex_escape() {
  printf '%s' "$1" \
    | sed \
      -e 's/\\/\\textbackslash{}/g' \
      -e 's/[&%$#_]/\\&/g' \
      -e 's/{/\\{/g' \
      -e 's/}/\\}/g'
}

is_positive() {
  local value="$1"
  [[ "${value}" == *"是"* || "${value}" == *"成功"* || "${value}" == *"通过"* || "${value}" == *"到达"* || "${value}" == *"yes"* || "${value}" == *"Yes"* || "${value}" == *"true"* || "${value}" == "1" ]]
}

if [[ ! -f "${RECORD_FILE}" ]]; then
  echo "[task1-sync] 缺少实验记录文件: ${RECORD_FILE}" >&2
  exit 1
fi

mkdir -p "$(dirname "${GENERATED_MD}")" "$(dirname "${GENERATED_TEX}")"

tmp_md="$(mktemp)"
tmp_tex="$(mktemp)"
trap 'rm -f "${tmp_md}" "${tmp_tex}"' EXIT

cat >"${tmp_md}" <<'EOF'
# 静态避障实验表

> 本文件由 `./run.sh task1-sync-report` 自动生成，数据来源为 `tasks/task1/EXPERIMENT_RECORD.md`。
> 如需修改实验数据，请优先修改实验记录原表，再重新运行同步命令。

| 编号 | 起点 | 目标点 | 主要经过障碍 | 是否到达 | 是否碰撞 | 是否成功 | 备注 |
|---|---|---|---|---|---|---|---|
EOF

cat >"${tmp_tex}" <<'EOF'
% 本文件由 ./run.sh task1-sync-report 自动生成。
% 数据来源：tasks/task1/EXPERIMENT_RECORD.md
% 如需修改实验数据，请优先修改实验记录原表，再重新运行同步命令。
EOF

in_table=false
rows=0
success_count=0

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

  start="$(normalize_cell "${c_start:-}")"
  goal="$(normalize_cell "${c_goal:-}")"
  obstacle="$(normalize_cell "${c_obstacle:-}")"
  arrived="$(normalize_cell "${c_arrived:-}")"
  collision="$(normalize_cell "${c_collision:-}")"
  success="$(normalize_cell "${c_success:-}")"
  note="$(normalize_cell "${c_note:-}")"

  rows=$((rows + 1))
  if is_positive "${success}" && ! is_positive "${collision}"; then
    success_count=$((success_count + 1))
  fi

  printf '| %s | %s | %s | %s | %s | %s | %s | %s |\n' \
    "${num}" "${start}" "${goal}" "${obstacle}" "${arrived}" "${collision}" "${success}" "${note}" >>"${tmp_md}"

  printf '  %s & %s & %s & %s & %s & %s & %s \\\\\n' \
    "$(latex_escape "${num}")" \
    "$(latex_escape "${start}")" \
    "$(latex_escape "${goal}")" \
    "$(latex_escape "${obstacle}")" \
    "$(latex_escape "${arrived}")" \
    "$(latex_escape "${collision}")" \
    "$(latex_escape "${success}")" >>"${tmp_tex}"
done < "${RECORD_FILE}"

if (( rows == 0 )); then
  echo "[task1-sync] 未解析到静态避障实验表，请检查 ${RECORD_FILE}" >&2
  exit 1
fi

{
  echo
  echo "成功率草稿："
  echo
  echo '```text'
  echo "成功次数：${success_count}"
  echo "总次数：${rows}"
  if (( rows > 0 )); then
    echo "成功率：$((success_count * 100 / rows))%"
  else
    echo "成功率：0%"
  fi
  echo '```'
  echo
  echo "说明：最终成功率以人工确认后的“是否成功/是否碰撞”字段为准。"
} >>"${tmp_md}"

mv "${tmp_md}" "${GENERATED_MD}"
printf '  \\bottomrule\n' >>"${tmp_tex}"
mv "${tmp_tex}" "${GENERATED_TEX}"
trap - EXIT

log "已生成: ${GENERATED_MD}"
log "已生成: ${GENERATED_TEX}"
log "解析到静态避障实验 ${rows} 行，当前成功次数 ${success_count}。"

if (( rows < 10 )); then
  echo "[task1-sync] WARN: 静态避障实验少于 10 行，最终提交前需要补齐。" >&2
fi
