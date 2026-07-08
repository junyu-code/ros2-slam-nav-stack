#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

strict=false
run_piper_boundary=true

print_usage() {
  cat <<'EOF'
用法:
  ./run.sh real-preflight [--strict] [--skip-piper-boundary]

说明:
  无 GUI、无硬件实机部署前预检。它只读取配置、检查包边界和默认安全开关，
  不启动 Gazebo/RViz/Nav2，不连接底盘或机械臂。

选项:
  --strict               将 warning 也视为失败，适合真正上车前最后确认
  --skip-piper-boundary  跳过 Piper/task1 隔离边界检查
EOF
}

for arg in "$@"; do
  case "${arg}" in
    --strict)
      strict=true
      ;;
    --skip-piper-boundary)
      run_piper_boundary=false
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "[real-preflight] 未知参数: ${arg}" >&2
      print_usage >&2
      exit 2
      ;;
  esac
done

export REAL_PREFLIGHT_STRICT="${strict}"
export REAL_PREFLIGHT_RUN_PIPER_BOUNDARY="${run_piper_boundary}"

python3 - <<'PY'
import os
import socket
import subprocess
from pathlib import Path

try:
    import yaml
except Exception as exc:  # 只在本机缺少 PyYAML 时触发。
    print(f'[real-preflight] FAIL: 缺少 PyYAML，无法读取 ROS 参数 YAML: {exc}')
    raise SystemExit(1)


ROOT = Path.cwd()
STRICT = os.environ.get('REAL_PREFLIGHT_STRICT') == 'true'
RUN_PIPER_BOUNDARY = os.environ.get('REAL_PREFLIGHT_RUN_PIPER_BOUNDARY') == 'true'

errors = 0
warnings = 0


def ok(message):
    print(f'[real-preflight] OK: {message}')


def warn(message):
    global warnings
    warnings += 1
    print(f'[real-preflight] WARN: {message}')


def fail(message):
    global errors
    errors += 1
    print(f'[real-preflight] FAIL: {message}')


def read_yaml(path):
    try:
        return yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception as exc:
        fail(f'无法读取 YAML: {path}: {exc}')
        return {}


def params_from(path, node_name):
    data = read_yaml(path)
    params = data.get(node_name, {}).get('ros__parameters', {})
    if not isinstance(params, dict):
        fail(f'{path} 中缺少 {node_name}.ros__parameters')
        return {}
    return params


def require_path(path, description, executable=False):
    path = ROOT / path
    label = path.relative_to(ROOT)
    if executable:
        if path.is_file() and os.access(path, os.X_OK):
            ok(f'{description}: {label}')
        else:
            fail(f'缺少或不可执行 {description}: {label}')
    else:
        if path.exists():
            ok(f'{description}: {label}')
        else:
            fail(f'缺少 {description}: {label}')


def require_param(params, key, description):
    if key not in params:
        fail(f'缺少参数 {description}: {key}')
        return None
    ok(f'{description}: {key}={params[key]!r}')
    return params[key]


def require_bool(params, key, expected, description, severity='fail'):
    value = require_param(params, key, description)
    if value is None:
        return
    if bool(value) != expected:
        message = f'{description} 应为 {expected}，实际为 {value!r}'
        if severity == 'warn':
            warn(message)
        else:
            fail(message)


def check_range(params, key, low, high, description, severity='fail'):
    value = require_param(params, key, description)
    if value is None:
        return
    try:
        number = float(value)
    except (TypeError, ValueError):
        fail(f'{description} 不是数值: {key}={value!r}')
        return
    if not (low <= number <= high):
        message = f'{description} 超出建议范围 [{low}, {high}]: {number}'
        if severity == 'warn':
            warn(message)
        else:
            fail(message)


def command_output(command, timeout=12):
    return subprocess.run(
        command,
        cwd=ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )


def check_pkg(package):
    result = command_output(['ros2', 'pkg', 'prefix', package], timeout=8)
    if result.returncode == 0:
        ok(f'ROS 包可解析: {package} -> {result.stdout.strip()}')
    else:
        fail(f'ROS 包不可解析: {package}\n{result.stdout[-800:]}')


def check_file_contains(path, tokens, description):
    path = ROOT / path
    if not path.is_file():
        fail(f'缺少 {description}: {path.relative_to(ROOT)}')
        return
    text = path.read_text(encoding='utf-8', errors='ignore')
    missing = [token for token in tokens if token not in text]
    if missing:
        fail(f'{description} 缺少关键内容: {missing}')
    else:
        ok(f'{description} 包含关键接口: {path.relative_to(ROOT)}')


print(f'[real-preflight] 工作区: {ROOT}')
print(f'[real-preflight] strict={STRICT}, run_piper_boundary={RUN_PIPER_BOUNDARY}')

print('\n[real-preflight] 1/7 基础包、脚本和入口')
for package_dir in [
    'src/safe_cmd_bridge',
    'src/localization_guard',
    'src/cloud_relocalization',
    'src/rgbd_navigation_perception',
    'src/mission_behavior',
    'src/slam_nav_bringup',
]:
    require_path(package_dir, '实机/扩展相关包目录')

for script in [
    'scripts/start_safe_cmd_bridge.sh',
    'scripts/start_localization_guard.sh',
    'scripts/start_relocalization.sh',
    'scripts/start_relocalization_gicp.sh',
    'scripts/start_navigation_rgbd.sh',
    'scripts/start_robust_navigation.sh',
    'scripts/diagnose_runtime.sh',
    'scripts/task1_runtime_check.sh',
]:
    require_path(script, '统一脚本入口', executable=True)

for package in [
    'safe_cmd_bridge',
    'localization_guard',
    'cloud_relocalization',
    'rgbd_navigation_perception',
    'mission_behavior',
    'slam_nav_bringup',
]:
    check_pkg(package)

print('\n[real-preflight] 2/7 速度安全桥默认安全开关')
safe_params = params_from(ROOT / 'src/safe_cmd_bridge/config/safe_cmd_bridge.yaml', 'safe_cmd_bridge_node')
require_bool(safe_params, 'enable_topic_output', True, 'ROS topic 安全输出默认开启')
require_bool(safe_params, 'enable_udp_output', False, 'UDP 输出默认关闭')
require_bool(safe_params, 'enable_fault_stop', True, '定位故障停车默认开启')
require_bool(safe_params, 'send_zero_on_shutdown', True, '退出时发送零速度')
require_bool(safe_params, 'enable_feedback_watchdog', False, '底盘反馈看门狗默认关闭', severity='warn')
check_range(safe_params, 'max_vx', 0.05, 1.2, '前向速度上限')
check_range(safe_params, 'max_vy', 0.0, 0.8, '横向速度上限')
check_range(safe_params, 'max_wz', 0.05, 1.5, '角速度上限')
check_range(safe_params, 'command_timeout_sec', 0.05, 1.0, '速度指令超时')
check_range(safe_params, 'publish_rate_hz', 5.0, 100.0, '安全桥发布频率')
for key in [
    'feedback_topic',
    'feedback_fault_topic',
    'feedback_health_topic',
    'feedback_timeout_sec',
    'feedback_stall_timeout_sec',
    'min_command_speed_for_feedback',
    'min_feedback_speed',
]:
    require_param(safe_params, key, '底盘反馈闭环预留参数')

if safe_params.get('udp_host') in ('127.0.0.1', 'localhost', ''):
    warn('udp_host 仍是本机或空地址；真实底盘 UDP 输出前需要改成底盘控制端 IP')
else:
    ok(f'UDP 目标地址已填写为: {safe_params.get("udp_host")}')

print('\n[real-preflight] 3/7 定位健康监控')
guard_params = params_from(ROOT / 'src/localization_guard/config/localization_guard.yaml', 'localization_guard_node')
for key in ['odom_topic', 'cloud_topic', 'scan_topic', 'health_topic', 'fault_topic', 'diagnostic_topic']:
    require_param(guard_params, key, '定位健康监控话题')
for key in ['odom_timeout_sec', 'cloud_timeout_sec', 'scan_timeout_sec', 'fault_hold_sec']:
    check_range(guard_params, key, 0.05, 10.0, '定位健康监控时间阈值')
require_bool(guard_params, 'publish_zero_on_fault', False, '定位故障默认只监控不抢控制', severity='warn')
if guard_params.get('use_sim_time') is True:
    warn('localization_guard 默认 use_sim_time=true；实机运行时需要显式传 use_sim_time:=false')

print('\n[real-preflight] 4/7 PCD 辅助重定位边界')
relocal_params = params_from(ROOT / 'src/cloud_relocalization/config/icp_relocalization.yaml', 'icp_relocalization_node')
require_bool(relocal_params, 'publish_tf', False, '重定位默认不发布 map->odom')
require_bool(relocal_params, 'auto_align', False, '重定位默认不自动闭环')
method = require_param(relocal_params, 'registration_method', '点云配准后端')
if method not in ('icp', 'gicp', 'ndt'):
    fail(f'registration_method 不在 icp/gicp/ndt 内: {method!r}')
for key in ['fitness_score_threshold', 'max_result_translation_jump', 'max_result_yaw_jump', 'local_map_radius']:
    check_range(relocal_params, key, 0.01, 20.0, '重定位安全阈值')
map_pcd_path = relocal_params.get('map_pcd_path', '')
if map_pcd_path:
    pcd_path = Path(map_pcd_path).expanduser()
    if pcd_path.is_file():
        ok(f'默认重定位 PCD 存在: {pcd_path}')
    else:
        warn(f'默认 map_pcd_path 已填写但文件不存在: {pcd_path}')
else:
    warn('默认 map_pcd_path 为空；实机重定位前需要传入已验证的 PCD 地图')

print('\n[real-preflight] 5/7 RGB-D 松耦合与 Nav2 观测源')
require_path('src/rgbd_navigation_perception/launch/depth_obstacle_projector.launch.py', 'RGB-D 障碍物投影 launch')
require_path('src/rgbd_navigation_perception/src/depth_obstacle_projector.cpp', 'RGB-D 障碍物投影节点')
check_file_contains(
    'src/slam_nav_bringup/config/nav2_params_3d_rgbd.yaml',
    ['/visual_obstacles', 'ObstacleLayer'],
    'RGB-D Nav2 参数',
)
check_file_contains(
    'src/rgbd_navigation_perception/README.md',
    ['/nav_camera/d435i/depth/image_rect_raw', '/visual_obstacles'],
    'RGB-D 接口说明',
)

print('\n[real-preflight] 6/7 文档和长期路线')
for doc in [
    'README.md',
    'PROJECT_PROCESS.md',
    'tasks/task2/FUTURE_ROADMAP.md',
    'tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md',
    'tasks/task2/ROBUST_NAVIGATION_UPGRADE_PLAN.md',
    'tasks/task2/PIPER_MOBILE_MANIPULATION.md',
]:
    require_path(doc, '长期维护文档')
check_file_contains(
    'tasks/task2/ROBUST_NAVIGATION_UPGRADE_PLAN.md',
    ['safe_cmd_bridge', 'localization_guard', 'cloud_relocalization'],
    '鲁棒导航路线图',
)
check_file_contains(
    'tasks/task2/REAL_ROBOT_DEPLOYMENT_CHECKLIST.md',
    ['enable_udp_output', '/cmd_vel_safe', 'map -> odom', '/nav_camera'],
    '实机部署检查清单',
)

print('\n[real-preflight] 7/7 网络和 Piper 边界')
try:
    ok(f'主机名可读取: {socket.gethostname()}')
except Exception as exc:
    warn(f'读取主机名失败: {exc}')

ip_result = command_output(['ip', '-4', '-o', 'addr', 'show', 'scope', 'global'], timeout=5)
ip_lines = []
if ip_result.returncode == 0:
    for line in ip_result.stdout.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[1] != 'lo':
            ip_lines.append(f'{parts[1]} {parts[3]}')
if ip_lines:
    ok('检测到非 loopback IPv4:\n' + '\n'.join(ip_lines))
else:
    warn('未检测到非 loopback IPv4；实机 UDP 联调前需要确认 WSL/虚拟机网络模式和底盘网段')

if RUN_PIPER_BOUNDARY:
    boundary = ROOT / 'scripts/piper_boundary_check.sh'
    if not boundary.is_file() or not os.access(boundary, os.X_OK):
        fail('Piper 边界检查脚本缺失或不可执行: scripts/piper_boundary_check.sh')
    else:
        result = command_output(['bash', str(boundary)], timeout=60)
        print(result.stdout, end='')
        if result.returncode == 0:
            ok('Piper/task1 边界检查通过')
        else:
            fail(f'Piper/task1 边界检查失败\n{result.stdout[-2000:]}')
else:
    warn('已按参数跳过 Piper/task1 边界检查')

print()
print(f'[real-preflight] 结果: errors={errors}, warnings={warnings}')
if errors:
    print('[real-preflight] 结论: 实机部署前预检失败，请先处理 FAIL 项。')
    raise SystemExit(1)

if STRICT and warnings:
    print('[real-preflight] 结论: --strict 模式下 warning 也视为未通过。')
    raise SystemExit(1)

if warnings:
    print('[real-preflight] 结论: 基础边界可用，但上车前需要处理 WARN 项。')
else:
    print('[real-preflight] 结论: 基础边界良好，可以进入 dry-run 或实机分项联调。')
PY
