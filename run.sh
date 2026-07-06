#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS_DIR="${ROOT_DIR}/scripts"

show_help() {
  cat <<'EOF'
用法：
  ./run.sh                         打开交互菜单
  ./run.sh <命令> [参数...]         直接运行指定流程

常用命令：
  build                 编译工作区
  clean                 清理 ROS/Gazebo/RViz/Nav2 残留进程
  sim                   启动默认仿真（当前默认动态场地）
  sim-static            启动静态验收场地仿真
  sim-dynamic           启动动态障碍仿真
  sim-dynamic-rgbd      启动动态障碍 + RGB-D 仿真
  mapping               手动建图链路
  auto-mapping          自动探索建图链路
  teleop                键盘控制
  save-map [name]       保存 2D 栅格地图
  save-pcd [name]       保存 FAST-LIO PCD 地图
  nav                   默认导航
  nav-3d                3D 地形增强导航
  nav-rgbd              RGB-D 松耦合导航
  nav-full              动态障碍 + RGB-D + 3D 完整导航
  robust-nav            鲁棒导航入口
  relocalization        ICP/GICP/NDT 重定位入口
  relocalization-gicp   GICP 重定位快捷入口
  guard                 定位健康监控
  safe-bridge           速度安全桥
  diagnose              运行时诊断
  task1-check           task1 交付材料预检（不启动 GUI）
  setup-piper           准备 Piper 外部参考包
  setup-piper-moveit    准备 Piper 本地 MoveIt2 OMPL overlay（无需 sudo）
  piper-preflight       Piper 依赖预检（自动加载本地 MoveIt overlay）
  piper-sim             单独启动 Piper 假感知/假执行冒烟链路
  piper-moveit-plan     启动 Piper 项目侧 MoveIt2 plan-only 配置
  piper-plan-test       向 MoveIt2 发送一次 Piper plan-only 规划请求
  piper-moveit-smoke    一键启动 MoveIt2 plan-only 并发送规划冒烟请求
  piper-gazebo-smoke    一键启动 headless Gazebo 并检查官方 Piper 适配链

示例：
  ./run.sh sim-static
  ./run.sh auto-mapping
  ./run.sh save-pcd nav_test_static
  ./run.sh nav-full
  ./run.sh task1-check
  ./run.sh setup-piper-moveit
  ./run.sh piper-preflight
  ./run.sh piper-sim
  ./run.sh piper-moveit-plan
  ./run.sh piper-plan-test
  ./run.sh piper-moveit-smoke
  ./run.sh piper-gazebo-smoke
EOF
}

script_for_command() {
  case "$1" in
    build) echo "build.sh" ;;
    clean) echo "clean.sh" ;;
    sim|simulation) echo "start_simulation.sh" ;;
    sim-static|static-sim) echo "start_simulation_static.sh" ;;
    sim-dynamic) echo "start_simulation_dynamic.sh" ;;
    sim-dynamic-rgbd) echo "start_simulation_dynamic_rgbd.sh" ;;
    mapping|map) echo "start_mapping.sh" ;;
    auto-mapping|auto-map) echo "start_auto_mapping.sh" ;;
    teleop) echo "teleop.sh" ;;
    save-map) echo "save_map.sh" ;;
    save-pcd) echo "save_pcd_map.sh" ;;
    nav|navigation) echo "start_navigation.sh" ;;
    nav-3d) echo "start_navigation_3d.sh" ;;
    nav-rgbd) echo "start_navigation_rgbd.sh" ;;
    nav-full|full) echo "start_navigation_full.sh" ;;
    robust-nav|robust) echo "start_robust_navigation.sh" ;;
    relocalization|relocalize) echo "start_relocalization.sh" ;;
    relocalization-gicp|gicp) echo "start_relocalization_gicp.sh" ;;
    guard|localization-guard) echo "start_localization_guard.sh" ;;
    safe-bridge) echo "start_safe_cmd_bridge.sh" ;;
    diagnose) echo "diagnose_runtime.sh" ;;
    task1-check|task1-preflight|task1) echo "task1_preflight.sh" ;;
    setup-piper) echo "setup_piper_open_class.sh" ;;
    setup-piper-moveit|setup-piper-moveit-overlay) echo "setup_piper_moveit_overlay.sh" ;;
    piper-preflight|piper-check) echo "piper_preflight.sh" ;;
    piper-sim) echo "start_piper_sim.sh" ;;
    piper-moveit-plan|piper-moveit) echo "start_piper_moveit_plan.sh" ;;
    piper-plan-test|piper-plan-smoke) echo "piper_plan_smoke_test.sh" ;;
    piper-moveit-smoke|piper-smoke) echo "piper_moveit_smoke.sh" ;;
    piper-gazebo-smoke|piper-gazebo) echo "piper_gazebo_smoke.sh" ;;
    help|-h|--help) echo "__help__" ;;
    *) return 1 ;;
  esac
}

run_command() {
  local command_name="$1"
  shift || true

  local script_name
  if ! script_name="$(script_for_command "${command_name}")"; then
    echo "[run] 未知命令：${command_name}" >&2
    echo >&2
    show_help >&2
    exit 2
  fi

  if [[ "${script_name}" == "__help__" ]]; then
    show_help
    exit 0
  fi

  local script_path="${SCRIPTS_DIR}/${script_name}"
  if [[ ! -x "${script_path}" ]]; then
    echo "[run] 脚本不可执行或不存在：${script_path}" >&2
    exit 2
  fi

  exec "${script_path}" "$@"
}

show_menu() {
  cat <<'EOF'
请选择要运行的流程：
  1) clean              清理残留
  2) sim                启动默认仿真（当前默认动态场地）
  3) sim-static         启动静态验收场地
  4) mapping            手动建图
  5) auto-mapping       自动探索建图
  6) teleop             键盘控制
  7) save-map           保存 2D 栅格地图
  8) save-pcd           保存 PCD 地图
  9) nav                默认导航
 10) nav-3d             3D 地形增强导航
 11) nav-full           完整增强导航
 12) diagnose           运行时诊断
 13) task1-check        task1 交付材料预检
 14) build              编译工作区
  h) help               查看全部命令
  q) quit               退出
EOF
  printf "输入编号或命令："
}

if [[ $# -gt 0 ]]; then
  run_command "$@"
fi

show_menu
read -r choice

case "${choice}" in
  1) run_command clean ;;
  2) run_command sim ;;
  3) run_command sim-static ;;
  4) run_command mapping ;;
  5) run_command auto-mapping ;;
  6) run_command teleop ;;
  7)
    read -r -p "地图名 [nav_test_map]：" map_name
    run_command save-map "${map_name:-nav_test_map}"
    ;;
  8)
    read -r -p "PCD 名 [nav_test_static]：" pcd_name
    run_command save-pcd "${pcd_name:-nav_test_static}"
    ;;
  9) run_command nav ;;
  10) run_command nav-3d ;;
  11) run_command nav-full ;;
  12) run_command diagnose ;;
  13) run_command task1-check ;;
  14) run_command build ;;
  h|help) show_help ;;
  q|quit|"") exit 0 ;;
  *) run_command "${choice}" ;;
esac
