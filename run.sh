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
  real-preflight        实机部署前无 GUI/无硬件预检
  diagnose              运行时诊断
  task1-status          task1 当前剩余证据/下一步（不启动 GUI）
  task1-snapshot        生成 task1 当前证据状态快照 md（不启动 GUI）
  task1-check           task1 交付材料预检（不启动 GUI）
  task1-runtime-check   task1 运行时链路检查（不启动 GUI）
  task1-experiment-check task1 静态避障实验表/成功率检查（不启动 GUI）
  task1-delivery-check  task1 打包交付前自查（不启动 GUI）
  task1-package-preview task1 最终压缩包预览/可选创建
  task1-report-audit   task1 结课报告源文件/截图/PDF 审计（不启动 GUI）
  task1-build-report    编译 task1 LaTeX 结课报告 PDF
  task1-finalize        task1 最终交付编排（编译报告/strict 检查/可选打包）
  setup-piper           准备 Piper 外部参考包
  setup-piper-moveit    准备 Piper 本地 MoveIt2 OMPL overlay（无需 sudo）
  piper-safety-check    检查 Piper 实机前安全默认值和边界开关
  piper-preflight       Piper 依赖预检（自动加载本地 MoveIt overlay）
  piper-frame-audit     检查 Piper 官方 URDF 到项目 piper_* frame 映射
  piper-sim             单独启动 Piper 假感知/假执行冒烟链路
  piper-viz             启动 Piper RViz 可视化（官方 URDF + 假腕部相机）
  piper-moveit-plan     启动 Piper 项目侧 MoveIt2 plan-only 配置
  piper-official-frame-audit 审计 AgileX 官方 URDF 到项目侧 piper_* frame 的适配
  piper-moveit-config   审计 Piper 项目侧 MoveIt2 配置和官方 AgileX 映射
  piper-hand-eye-check   检查 Piper 腕部 RGB-D 手眼标定配置边界
  piper-hand-eye-gate    验证真实 pick 缺少手眼标定时会安全拒绝
  piper-base-stop-gate   验证真实运动缺少底盘停止确认时会安全拒绝
  piper-plan-test       向 MoveIt2 发送一次 Piper plan-only 规划请求
  piper-moveit-smoke    一键启动 MoveIt2 plan-only 并发送规划冒烟请求
  piper-tf-smoke        一键验证 Piper 运行时 TF 链和 task1 TF 隔离边界
  piper-namespace-smoke 一键验证 Piper runtime topic/action 不污染 Nav2 或 /nav_camera
  piper-gazebo-smoke    一键启动 headless Gazebo 并检查官方 Piper 适配链
  piper-task-smoke      一键验证 Piper 假感知、抓取候选和 pick/place action
  piper-mobile-sequence 一键验证移动操作组合入口的假相机、停车、pick/place 顺序
  piper-mission-demo    一键验证 mission_behavior 只通过 /piper/task/* 调用 Piper
  piper-control-smoke   一键验证 Piper 控制桥 owner/enable/estop 边界
  piper-real-readiness  Piper 实机接入前状态报告（默认不要求 ready）
  piper-real-dry-run    一键验证 Piper 实机入口默认安全拒绝真实执行
  piper-learning-smoke  一键验证 Piper 学习层抓取候选排序旁路
  piper-ranked-gate     验证任务层显式打开后才消费 ranked 抓取候选
  piper-size-check      检查 Piper 外部依赖/数据/权重没有进入 Git 跟踪
  piper-static-check    Piper 静态配置验收（不启动 Gazebo/MoveIt2/真实硬件）
  piper-full-smoke      顺序运行 Piper 安全、边界、体积、手眼、TF、Gazebo、任务、学习、ranked、MoveIt2 烟测
  piper-boundary-check  检查 Piper 未泄漏进 task1/Nav2 或 /nav_camera

示例：
  ./run.sh sim-static
  ./run.sh auto-mapping
  ./run.sh save-pcd nav_test_static
  ./run.sh nav-full
  ./run.sh task1-status
  ./run.sh task1-snapshot
  ./run.sh task1-check
  ./run.sh task1-runtime-check nav
  ./run.sh task1-experiment-check
  ./run.sh task1-delivery-check
  ./run.sh task1-package-preview
  ./run.sh task1-report-audit
  ./run.sh task1-build-report
  ./run.sh task1-finalize
  ./run.sh real-preflight
  ./run.sh piper-safety-check
  ./run.sh piper-frame-audit
  ./run.sh setup-piper-moveit
  ./run.sh piper-preflight
  ./run.sh piper-sim
  ./run.sh piper-viz
  ./run.sh piper-moveit-plan
  ./run.sh piper-official-frame-audit
  ./run.sh piper-moveit-config
  ./run.sh piper-hand-eye-check
  ./run.sh piper-hand-eye-gate
  ./run.sh piper-base-stop-gate
  ./run.sh piper-plan-test
  ./run.sh piper-moveit-smoke
  ./run.sh piper-tf-smoke
  ./run.sh piper-namespace-smoke
  ./run.sh piper-gazebo-smoke
  ./run.sh piper-task-smoke
  ./run.sh piper-mobile-sequence
  ./run.sh piper-mission-demo
  ./run.sh piper-control-smoke
  ./run.sh piper-real-readiness
  ./run.sh piper-real-dry-run
  ./run.sh piper-learning-smoke
  ./run.sh piper-ranked-gate
  ./run.sh piper-size-check
  ./run.sh piper-static-check
  ./run.sh piper-full-smoke
  ./run.sh piper-boundary-check
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
    real-preflight|real-check|deploy-check) echo "real_preflight.sh" ;;
    diagnose) echo "diagnose_runtime.sh" ;;
    task1-status|task1-next|task1-todo|status-task1) echo "task1_status.sh" ;;
    task1-snapshot|task1-state|task1-progress) echo "task1_snapshot.sh" ;;
    task1-check|task1-preflight|task1) echo "task1_preflight.sh" ;;
    task1-runtime-check|task1-runtime|runtime-check) echo "task1_runtime_check.sh" ;;
    task1-experiment-check|task1-experiment|experiment-check|experiment) echo "task1_experiment_check.sh" ;;
    task1-delivery-check|task1-delivery|delivery-check) echo "task1_delivery_check.sh" ;;
    task1-package-preview|task1-package|package-preview) echo "task1_package_preview.sh" ;;
    task1-report-audit|task1-report-check|report-audit) echo "task1_report_audit.sh" ;;
    task1-build-report|task1-report|build-report|report-build) echo "build_task1_report.sh" ;;
    task1-finalize|task1-final|finalize-task1|task1-submit-check) echo "task1_finalize.sh" ;;
    setup-piper) echo "setup_piper_open_class.sh" ;;
    setup-piper-moveit|setup-piper-moveit-overlay) echo "setup_piper_moveit_overlay.sh" ;;
    piper-safety-check|piper-safety) echo "piper_safety_check.sh" ;;
    piper-preflight|piper-check) echo "piper_preflight.sh" ;;
    piper-frame-audit|piper-frame|piper-official-frame|piper-official-frame-audit) echo "piper_official_frame_audit.sh" ;;
    piper-sim) echo "start_piper_sim.sh" ;;
    piper-viz|piper-visualization|piper-rviz) echo "start_piper_visualization.sh" ;;
    piper-moveit-plan|piper-moveit) echo "start_piper_moveit_plan.sh" ;;
    piper-moveit-config|piper-moveit-config-audit) echo "piper_moveit_config_audit.sh" ;;
    piper-hand-eye-check|piper-hand-eye|piper-calibration-check) echo "piper_hand_eye_check.sh" ;;
    piper-hand-eye-gate|piper-hand-eye-smoke|piper-calibration-gate) echo "piper_hand_eye_gate_smoke.sh" ;;
    piper-base-stop-gate|piper-base-stop|piper-nav-pause-gate) echo "piper_base_stop_gate_smoke.sh" ;;
    piper-plan-test|piper-plan-smoke) echo "piper_plan_smoke_test.sh" ;;
    piper-moveit-smoke|piper-smoke) echo "piper_moveit_smoke.sh" ;;
    piper-tf-smoke|piper-tf) echo "piper_tf_smoke.sh" ;;
    piper-namespace-smoke|piper-namespace|piper-ns) echo "piper_namespace_smoke.sh" ;;
    piper-gazebo-smoke|piper-gazebo) echo "piper_gazebo_smoke.sh" ;;
    piper-task-smoke|piper-task) echo "piper_task_smoke.sh" ;;
    piper-mobile-sequence|piper-mobile-sequence-smoke|piper-mobile-task) echo "piper_mobile_sequence_smoke.sh" ;;
    piper-mission-demo|piper-mission-smoke|mission-piper-demo) echo "piper_mission_demo_smoke.sh" ;;
    piper-control-smoke|piper-control) echo "piper_control_smoke.sh" ;;
    piper-real-readiness|piper-real-ready|piper-readiness) echo "piper_real_readiness.sh" ;;
    piper-real-dry-run|piper-real-dry|piper-real-smoke) echo "piper_real_dry_run.sh" ;;
    piper-learning-smoke|piper-learning) echo "piper_learning_smoke.sh" ;;
    piper-ranked-gate|piper-ranked-candidate|piper-ranked-smoke) echo "piper_ranked_candidate_gate_smoke.sh" ;;
    piper-size-check|piper-size|piper-repo-size) echo "piper_repo_size_check.sh" ;;
    piper-static-check|piper-static|piper-config-check|piper-acceptance) echo "piper_static_acceptance.sh" ;;
    piper-full-smoke|piper-full|piper-all-smoke) echo "piper_full_smoke.sh" ;;
    piper-boundary-check|piper-boundary) echo "piper_boundary_check.sh" ;;
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
  1) clean --dry-run    预览将清理的进程/共享内存
  2) clean              清理残留
  3) sim-static         启动静态验收场地
  4) sim-dynamic        启动动态障碍场地
  5) sim-dynamic-rgbd   启动动态障碍 + RGB-D 场地
  6) mapping            手动建图
  7) auto-mapping       自动探索建图
  8) teleop             键盘控制
  9) save-map           保存 2D 栅格地图
 10) save-pcd           保存 PCD 地图
 11) nav                默认导航
 12) nav-3d             3D 地形增强导航
 13) nav-rgbd           RGB-D 松耦合导航
 14) nav-full           动态障碍 + RGB-D + 3D 完整导航
 15) runtime mapping    检查建图运行时链路
 16) runtime nav        检查导航运行时链路
 17) diagnose           运行时诊断
 18) task1-status       查看 task1 剩余证据和下一步
 19) task1-snapshot     生成 task1 当前证据状态快照 md
 20) task1-check        task1 交付材料预检
 21) experiment-check   task1 静态避障实验表检查
 22) task1-delivery     task1 打包交付前自查
 23) package-preview    task1 压缩包预览
 24) report-audit       task1 结课报告源文件/截图/PDF 审计
 25) build-report       编译 task1 结课报告 PDF
 26) task1-finalize     task1 最终交付编排
 27) real-preflight     实机部署前预检
 28) build              编译工作区
  h) help               查看全部命令
  q) quit               退出

也可以直接输入 help 中的任意命令，例如 nav-full、task1-runtime-check nav 或 piper-safety-check。
EOF
  printf "输入编号或命令："
}

if [[ $# -gt 0 ]]; then
  run_command "$@"
fi

show_menu
read -r choice

case "${choice}" in
  1) run_command clean --dry-run ;;
  2) run_command clean ;;
  3) run_command sim-static ;;
  4) run_command sim-dynamic ;;
  5) run_command sim-dynamic-rgbd ;;
  6) run_command mapping ;;
  7) run_command auto-mapping ;;
  8) run_command teleop ;;
  9)
    read -r -p "地图名 [nav_test_map]：" map_name
    run_command save-map "${map_name:-nav_test_map}"
    ;;
  10)
    read -r -p "PCD 名 [nav_test_static]：" pcd_name
    run_command save-pcd "${pcd_name:-nav_test_static}"
    ;;
  11) run_command nav ;;
  12) run_command nav-3d ;;
  13) run_command nav-rgbd ;;
  14) run_command nav-full ;;
  15) run_command task1-runtime-check mapping ;;
  16) run_command task1-runtime-check nav ;;
  17) run_command diagnose ;;
  18) run_command task1-status ;;
  19) run_command task1-snapshot ;;
  20) run_command task1-check ;;
  21) run_command task1-experiment-check ;;
  22) run_command task1-delivery-check ;;
  23) run_command task1-package-preview ;;
  24) run_command task1-report-audit ;;
  25) run_command task1-build-report ;;
  26) run_command task1-finalize ;;
  27) run_command real-preflight ;;
  28) run_command build ;;
  h|help) show_help ;;
  q|quit|"") exit 0 ;;
  *) run_command "${choice}" ;;
esac
