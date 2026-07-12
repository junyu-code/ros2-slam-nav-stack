#!/usr/bin/env bash
set -euo pipefail

# UI 图形自检：不启动 Gazebo/RViz，只检查图形入口和关键环境变量。
echo "[ui-gui-check] 图形环境变量"
echo "DISPLAY=${DISPLAY:-<empty>}"
echo "WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<empty>}"
echo "XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-<empty>}"
echo "WSL_DISTRO_NAME=${WSL_DISTRO_NAME:-<empty>}"

echo
echo "[ui-gui-check] ROS 覆盖层"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  echo "OK /opt/ros/humble/setup.bash"
  # 加载 ROS 环境后再检查 ros2/rviz2，避免误报命令不存在。
  set +u
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "MISSING /opt/ros/humble/setup.bash"
fi

if [[ -f install/setup.bash ]]; then
  echo "OK install/setup.bash"
  set +u
  source install/setup.bash
  set -u
else
  echo "MISSING install/setup.bash"
fi

echo
echo "[ui-gui-check] 可执行文件"
for cmd in gazebo gzclient gzserver rviz2 ros2; do
  if command -v "${cmd}" >/dev/null 2>&1; then
    echo "OK ${cmd}: $(command -v "${cmd}")"
  else
    echo "MISSING ${cmd}"
  fi
done

echo
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  echo "[ui-gui-check] FAIL: 当前进程看不到 DISPLAY/WAYLAND_DISPLAY，Gazebo/RViz 图形窗口不会弹出。"
  echo "[ui-gui-check] 建议从 WSL Ubuntu 终端运行 ./run.sh ui，而不是从 Windows Python/PowerShell 后台启动 UI。"
  exit 1
fi

echo "[ui-gui-check] OK: 当前环境具备图形显示变量。"
