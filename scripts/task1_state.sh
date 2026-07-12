#!/usr/bin/env bash

# 统一读取 Task1 人工状态，避免各检查脚本自行推断“是否完成”。
TASK1_STATE_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK1_WORKSPACE_DIR="$(cd "${TASK1_STATE_SCRIPT_DIR}/.." && pwd)"
TASK1_STATE_FILE="${TASK1_STATE_FILE:-${TASK1_WORKSPACE_DIR}/tasks/task1/task1.env}"

task1_load_state() {
  TASK1_STATE="in_progress"
  TASK1_NAVIGATION_SCHEME="pending"
  TASK1_EVIDENCE_STATE="provisional"

  if [[ ! -f "${TASK1_STATE_FILE}" ]]; then
    echo "[task1-state] 缺少状态文件: ${TASK1_STATE_FILE}" >&2
    return 1
  fi

  # 状态文件只允许简单的 KEY=value，由仓库维护者人工修改。
  source "${TASK1_STATE_FILE}"

  case "${TASK1_STATE}" in
    in_progress|ready) ;;
    *)
      echo "[task1-state] TASK1_STATE 非法: ${TASK1_STATE}" >&2
      return 1
      ;;
  esac

  case "${TASK1_NAVIGATION_SCHEME}" in
    pending|nav_2d|nav_3d|nav_full) ;;
    *)
      echo "[task1-state] TASK1_NAVIGATION_SCHEME 非法: ${TASK1_NAVIGATION_SCHEME}" >&2
      return 1
      ;;
  esac

  case "${TASK1_EVIDENCE_STATE}" in
    provisional|final) ;;
    *)
      echo "[task1-state] TASK1_EVIDENCE_STATE 非法: ${TASK1_EVIDENCE_STATE}" >&2
      return 1
      ;;
  esac
}

task1_state_issues() {
  if [[ "${TASK1_STATE}" != "ready" ]]; then
    echo "Task1 仍处于 ${TASK1_STATE}，两份作业尚未进入最终交付态"
  fi
  if [[ "${TASK1_NAVIGATION_SCHEME}" == "pending" ]]; then
    echo "导航方案仍为 pending，需要在 nav_2d、nav_3d、nav_full 中冻结一个主方案"
  fi
  if [[ "${TASK1_EVIDENCE_STATE}" != "final" ]]; then
    echo "现有证据仍为 ${TASK1_EVIDENCE_STATE}，最终方案冻结后需要重新完成正式实验"
  fi
}

task1_state_is_ready() {
  [[ "${TASK1_STATE}" == "ready" &&
     "${TASK1_NAVIGATION_SCHEME}" != "pending" &&
     "${TASK1_EVIDENCE_STATE}" == "final" ]]
}

task1_print_state() {
  echo "TASK1_STATE=${TASK1_STATE}"
  echo "TASK1_NAVIGATION_SCHEME=${TASK1_NAVIGATION_SCHEME}"
  echo "TASK1_EVIDENCE_STATE=${TASK1_EVIDENCE_STATE}"
}
