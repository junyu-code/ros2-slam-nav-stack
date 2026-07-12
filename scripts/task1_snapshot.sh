#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

OUTPUT_PATH="artifacts/task1/status.txt"
WRITE_FILE=true

usage() {
  cat <<'EOF'
用法：
  ./run.sh task1-snapshot [--stdout] [--output <path>]

说明：
  保存 task1-status 的纯文本输出，不启动 ROS 或 GUI。
  默认写入被 Git 忽略的 artifacts/task1/status.txt。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --stdout)
      WRITE_FILE=false
      ;;
    --output)
      shift
      OUTPUT_PATH="${1:?--output 需要路径}"
      ;;
    -h|--help|help)
      usage
      exit 0
      ;;
    *)
      echo "[task1-snapshot] 未知参数: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ "${WRITE_FILE}" == "false" ]]; then
  exec "${SCRIPT_DIR}/task1_status.sh"
fi

mkdir -p "$(dirname "${OUTPUT_PATH}")"
tmp_output="$(mktemp)"
trap 'rm -f "${tmp_output}"' EXIT

set +e
"${SCRIPT_DIR}/task1_status.sh" 2>&1 | tee "${tmp_output}"
status=${PIPESTATUS[0]}
set -e

{
  echo "生成时间: $(date '+%Y-%m-%d %H:%M:%S %z')"
  echo "生成提交: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo
  cat "${tmp_output}"
} >"${OUTPUT_PATH}"

echo "[task1-snapshot] 已写入本地快照: ${OUTPUT_PATH}"
exit "${status}"
