#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

show_status=true

print_usage() {
  cat <<'EOF'
用法：
  ./run.sh task1               显示 Task1 主流程和精简状态
  ./run.sh task1 --no-status   只显示主流程
  ./run.sh task1 --full-status 显示主流程后输出完整状态
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-status)
      show_status=false
      ;;
    --full-status)
      show_status=full
      ;;
    -h|--help|help)
      print_usage
      exit 0
      ;;
    *)
      echo "[task1] 未知参数：$1" >&2
      print_usage >&2
      exit 2
      ;;
  esac
  shift
done

cat <<'EOF'
Task1 主流程

  1. 静态仿真    ./run.sh sim-static
       |
       +-- 建图   ./run.sh mapping + ./run.sh teleop
       |            -> ./run.sh save-map nav_test_map
       |
       +-- 候选 A ./run.sh nav
       +-- 候选 B ./run.sh nav-3d
       `-- 候选 C ./run.sh nav-full

  2. 冻结方案    更新 tasks/task1/task1.env
  3. 正式实验    使用冻结方案重新完成 10 次静态导航
  4. 最终交付    ./run.sh task1-finalize

关键规则：
  - 建图和导航是两个独立阶段；切换阶段前先运行 ./run.sh clean。
  - sim-static 只提供场地、机器人和传感器，不会自动启动建图或 Nav2。
  - mapping 生成地图；导航候选读取已经保存的 nav_test_map，不会更新地图文件。
  - 当前首选候选是 nav，但方案仍为 pending。
  - 现有 10/10 结果是 provisional 基线；最终方案冻结后必须重跑。
  - 80% 成功率只统计 10 次静态避障；动态障碍物属于扩展演示。

课程要求、方案比较、运行顺序和两份作业入口：tasks/task1/README.md
EOF

if [[ "${show_status}" == "false" ]]; then
  exit 0
fi

echo
echo "当前状态"
if [[ "${show_status}" == "full" ]]; then
  "${SCRIPT_DIR}/task1_status.sh"
else
  set +e
  "${SCRIPT_DIR}/task1_status.sh" --brief
  status=$?
  set -e
  if (( status != 0 )); then
    echo "[task1] 存在结构性缺失；请运行 ./run.sh task1-check 查看详情。"
  fi
fi
