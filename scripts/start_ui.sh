#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FRONTEND_DIR="${WORKSPACE_DIR}/ui/navigation"
BRIDGE="${FRONTEND_DIR}/bridge/ros2_nav_ws_bridge.py"
ROS_DISTRO="${ROS_DISTRO:-humble}"

# 新变量优先；旧 SLAM_NAV_NAV_UI_* 名称仅用于兼容已有部署。
HOST="${SLAM_NAV_UI_HOST:-${SLAM_NAV_NAV_UI_HOST:-127.0.0.1}}"
PORT="${SLAM_NAV_UI_PORT:-${SLAM_NAV_NAV_UI_PORT:-8765}}"
CAMERA_TOPIC="${SLAM_NAV_CAMERA_TOPIC:-/nav_camera/color/image_raw}"
NAV_DEPTH_TOPIC="${SLAM_NAV_DEPTH_TOPIC:-/nav_camera/depth/image_raw}"
PIPER_COLOR_TOPIC="${SLAM_NAV_PIPER_COLOR_TOPIC:-/piper/arm_camera/color/image_raw}"
PIPER_DEPTH_TOPIC="${SLAM_NAV_PIPER_DEPTH_TOPIC:-/piper/arm_camera/depth/image_raw}"
NAVIGATION_READY_TOPIC="${SLAM_NAV_NAVIGATION_READY_TOPIC:-/navigation_ready}"
DISABLE_CAMERA="${SLAM_NAV_UI_DISABLE_CAMERA:-${SLAM_NAV_NAV_UI_DISABLE_CAMERA:-0}}"
DISABLE_PIPER_CAMERA="${SLAM_NAV_UI_DISABLE_PIPER_CAMERA:-0}"
REBUILD_FRONTEND="${SLAM_NAV_UI_REBUILD:-${SLAM_NAV_NAV_UI_REBUILD:-0}}"

show_help() {
  cat <<'EOF'
用法：
  ./run.sh ui [选项]

选项：
  --host <地址>          监听地址，默认 127.0.0.1
  --port <端口>          Web 页面、REST API 与 WebSocket 端口，默认 8765
  --camera-topic <话题>  导航 RGB 图像话题
  --nav-depth-topic <话题> 导航深度图话题
  --piper-color-topic <话题> Piper 腕部 RGB 图像话题
  --piper-depth-topic <话题> Piper 腕部深度图话题
  --navigation-ready-topic <话题> 导航就绪 Bool 话题
  --no-camera            不订阅导航 RGB-D 相机
  --no-piper-camera      不订阅 Piper 腕部 RGB-D 相机
  --rebuild              使用 Vite 支持的 Linux Node.js 重新构建前端
  -h, --help             显示帮助

兼容入口 ./run.sh nav-ui 使用相同选项并启动同一界面。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --host)
      HOST="${2:?--host 需要一个地址}"
      shift 2
      ;;
    --host=*)
      HOST="${1#--host=}"
      shift
      ;;
    --port)
      PORT="${2:?--port 需要一个端口}"
      shift 2
      ;;
    --port=*)
      PORT="${1#--port=}"
      shift
      ;;
    --camera-topic)
      CAMERA_TOPIC="${2:?--camera-topic 需要一个话题名}"
      shift 2
      ;;
    --camera-topic=*)
      CAMERA_TOPIC="${1#--camera-topic=}"
      shift
      ;;
    --nav-depth-topic)
      NAV_DEPTH_TOPIC="${2:?--nav-depth-topic 需要一个话题名}"
      shift 2
      ;;
    --nav-depth-topic=*)
      NAV_DEPTH_TOPIC="${1#--nav-depth-topic=}"
      shift
      ;;
    --piper-color-topic)
      PIPER_COLOR_TOPIC="${2:?--piper-color-topic 需要一个话题名}"
      shift 2
      ;;
    --piper-color-topic=*)
      PIPER_COLOR_TOPIC="${1#--piper-color-topic=}"
      shift
      ;;
    --piper-depth-topic)
      PIPER_DEPTH_TOPIC="${2:?--piper-depth-topic 需要一个话题名}"
      shift 2
      ;;
    --piper-depth-topic=*)
      PIPER_DEPTH_TOPIC="${1#--piper-depth-topic=}"
      shift
      ;;
    --navigation-ready-topic)
      NAVIGATION_READY_TOPIC="${2:?--navigation-ready-topic 需要一个话题名}"
      shift 2
      ;;
    --navigation-ready-topic=*)
      NAVIGATION_READY_TOPIC="${1#--navigation-ready-topic=}"
      shift
      ;;
    --no-camera)
      DISABLE_CAMERA=1
      shift
      ;;
    --no-piper-camera)
      DISABLE_PIPER_CAMERA=1
      shift
      ;;
    --rebuild)
      REBUILD_FRONTEND=1
      shift
      ;;
    -h|--help)
      show_help
      exit 0
      ;;
    *)
      echo "[ui] 未知参数：$1" >&2
      show_help >&2
      exit 2
      ;;
  esac
done

if [[ -z "${HOST}" ]]; then
  echo "[ui] 监听地址不能为空。" >&2
  exit 2
fi

if [[ ! "${PORT}" =~ ^[0-9]+$ ]] || (( PORT < 1 || PORT > 65535 )); then
  echo "[ui] 端口必须是 1-65535 之间的整数：${PORT}" >&2
  exit 2
fi

CHECK_HOST="${HOST}"
if [[ "${CHECK_HOST}" == "0.0.0.0" || "${CHECK_HOST}" == "::" ]]; then
  CHECK_HOST="127.0.0.1"
fi

port_is_open() {
  python3 - "${CHECK_HOST}" "${PORT}" <<'PY'
import socket
import sys

try:
    with socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=0.25):
        pass
except OSError:
    raise SystemExit(1)
raise SystemExit(0)
PY
}

unified_ui_is_running() {
  python3 - "${CHECK_HOST}" "${PORT}" <<'PY'
import json
import sys
import urllib.request

host, port = sys.argv[1], int(sys.argv[2])
url_host = f"[{host}]" if ":" in host else host
try:
    with urllib.request.urlopen(f"http://{url_host}:{port}/api/state", timeout=5.0) as response:
        data = json.loads(response.read().decode("utf-8"))
except Exception:
    raise SystemExit(1)

required = ("flows", "health", "active", "history", "operator")
raise SystemExit(0 if isinstance(data, dict) and all(key in data for key in required) else 1)
PY
}

if port_is_open; then
  if unified_ui_is_running; then
    echo "[ui] SLAM Nav Web 主界面已在运行：http://${CHECK_HOST}:${PORT}/"
    exit 0
  fi
  echo "[ui] 端口已被其他程序占用：${HOST}:${PORT}" >&2
  echo "[ui] 如需并行启动，请显式指定其他端口，例如：./run.sh ui --port 8766" >&2
  exit 98
fi

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "[ui] 找不到 ROS 环境：/opt/ros/${ROS_DISTRO}/setup.bash" >&2
  exit 1
fi

if [[ ! -f "${BRIDGE}" ]]; then
  echo "[ui] 找不到 SLAM Nav Web bridge：${BRIDGE}" >&2
  exit 1
fi

is_windows_tool() {
  local tool_path="$1"
  local resolved_path
  resolved_path="$(readlink -f -- "${tool_path}" 2>/dev/null || true)"
  if [[ -n "${resolved_path}" ]]; then
    tool_path="${resolved_path}"
  fi
  case "${tool_path}" in
    /mnt/[a-zA-Z]/*|*.exe|*.cmd) return 0 ;;
    *) return 1 ;;
  esac
}

require_linux_node() {
  local node_path npm_path node_platform node_version major minor
  node_path="$(command -v node 2>/dev/null || true)"
  npm_path="$(command -v npm 2>/dev/null || true)"

  if [[ -z "${node_path}" || -z "${npm_path}" ]] \
    || is_windows_tool "${node_path}" \
    || is_windows_tool "${npm_path}"; then
    echo "[ui] 无法构建前端：WSL 中需要 Vite 支持的 Linux Node.js 和 Linux npm。" >&2
    if [[ -n "${npm_path}" ]] && is_windows_tool "${npm_path}"; then
      echo "[ui] 当前检测到的是 Windows npm：${npm_path}" >&2
    fi
    echo "[ui] 请安装 Linux Node.js 20.19+ 或 22.12+；不要从该工作区调用 Windows npm。" >&2
    exit 1
  fi

  node_platform="$(node -p 'process.platform' 2>/dev/null || true)"
  if [[ "${node_platform}" != "linux" ]]; then
    echo "[ui] 无法构建前端：当前 Node.js 不是 Linux 版本（platform=${node_platform:-unknown}）。" >&2
    echo "[ui] 请在 WSL/Ubuntu 内安装 Linux Node.js 20.19+ 或 22.12+ 和 Linux npm。" >&2
    exit 1
  fi

  node_version="$(node --version 2>/dev/null || true)"
  node_version="${node_version#v}"
  if [[ ! "${node_version}" =~ ^([0-9]+)\.([0-9]+)(\.[0-9]+)?([+-].*)?$ ]]; then
    echo "[ui] 无法识别 Node.js 版本：${node_version:-unknown}" >&2
    exit 1
  fi
  major="${BASH_REMATCH[1]}"
  minor="${BASH_REMATCH[2]}"
  if ! (( (major == 20 && minor >= 19) || (major == 22 && minor >= 12) || major >= 23 )); then
    echo "[ui] Vite 不支持当前 Node.js：v${node_version}；要求 ^20.19.0 或 >=22.12.0。" >&2
    exit 1
  fi
}

# 默认直接使用仓库内已构建页面；仅缺少产物或显式要求时依赖 Node.js。
if [[ "${REBUILD_FRONTEND}" == "1" || ! -f "${FRONTEND_DIR}/dist/index.html" ]]; then
  require_linux_node
  echo "[ui] 正在构建 SLAM Nav Web 前端..."
  if [[ -d "${FRONTEND_DIR}/node_modules" ]]; then
    (cd "${FRONTEND_DIR}" && npm run build)
  else
    (cd "${FRONTEND_DIR}" && npm ci && npm run build)
  fi
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi
set -u

echo "[ui] SLAM Nav Web 主界面：http://${CHECK_HOST}:${PORT}/"
echo "[ui] Qt/RViz Operator 可从主界面的专业界面入口打开，窗口显示在工作区主机桌面。"
echo "[ui] 导航 RGB-D：${CAMERA_TOPIC} / ${NAV_DEPTH_TOPIC}"
echo "[ui] Piper 腕部 RGB-D：${PIPER_COLOR_TOPIC} / ${PIPER_DEPTH_TOPIC}"
echo "[ui] 导航就绪：${NAVIGATION_READY_TOPIC}"
echo "[ui] 地图/路径：${SLAM_NAV_MAP_TOPIC:-/map} / ${SLAM_NAV_GLOBAL_PLAN_TOPIC:-/plan} / ${SLAM_NAV_LOCAL_PLAN_TOPIC:-/local_plan}"

ARGS=(
  --host "${HOST}"
  --port "${PORT}"
  --topic "${CAMERA_TOPIC}"
  --nav-depth-topic "${NAV_DEPTH_TOPIC}"
  --piper-color-topic "${PIPER_COLOR_TOPIC}"
  --piper-depth-topic "${PIPER_DEPTH_TOPIC}"
  --navigation-ready-topic "${NAVIGATION_READY_TOPIC}"
  --static-dir "${FRONTEND_DIR}/dist"
  --map-topic "${SLAM_NAV_MAP_TOPIC:-/map}"
  --global-costmap-topic "${SLAM_NAV_GLOBAL_COSTMAP_TOPIC:-/global_costmap/costmap}"
  --local-costmap-topic "${SLAM_NAV_LOCAL_COSTMAP_TOPIC:-/local_costmap/costmap}"
  --global-plan-topic "${SLAM_NAV_GLOBAL_PLAN_TOPIC:-/plan}"
  --local-plan-topic "${SLAM_NAV_LOCAL_PLAN_TOPIC:-/local_plan}"
  --map-frame "${SLAM_NAV_MAP_FRAME:-map}"
  --robot-frames "${SLAM_NAV_ROBOT_FRAMES:-base_footprint,base_link}"
)

if [[ "${DISABLE_CAMERA}" == "1" ]]; then
  ARGS+=(--disable-camera)
fi

if [[ "${DISABLE_PIPER_CAMERA}" == "1" ]]; then
  ARGS+=(--disable-piper-camera)
fi

exec python3 "${BRIDGE}" "${ARGS[@]}"
