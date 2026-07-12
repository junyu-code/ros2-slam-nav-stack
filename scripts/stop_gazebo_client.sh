#!/usr/bin/env bash
set -euo pipefail

# 单独关闭 Gazebo 图形客户端，保留 gzserver 后端仿真继续运行。
if ! pgrep -x gzclient >/dev/null 2>&1; then
  echo "[gazebo-client-stop] 未发现正在运行的 gzclient。"
  exit 0
fi

echo "[gazebo-client-stop] 正在请求 gzclient 退出。"
pkill -TERM -x gzclient || true
sleep 2

if pgrep -x gzclient >/dev/null 2>&1; then
  echo "[gazebo-client-stop] gzclient 未正常退出，强制结束。"
  pkill -KILL -x gzclient || true
else
  echo "[gazebo-client-stop] gzclient 已关闭。"
fi
