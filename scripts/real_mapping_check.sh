#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}/.."
source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

duration="5"

print_usage() {
  cat <<'EOF'
用法:
  ./run.sh real-mapping-check [--duration 秒] [diagnose_runtime.py 其他参数...]

说明:
  实机建图链路诊断，不要求 /clock，重点检查：
  /livox/lidar -> /livox/imu -> /imu/data
  -> /cloud_registered -> /Odometry -> /scan -> /map
EOF
}

args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      duration="${2:?--duration 需要秒数}"
      shift 2
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      args+=("$1")
      shift
      ;;
  esac
done

exec python3 src/slam_nav_bringup/scripts/diagnose_runtime.py \
  --real \
  --duration "${duration}" \
  --require-map \
  --skip-costmap-checks \
  "${args[@]}"
