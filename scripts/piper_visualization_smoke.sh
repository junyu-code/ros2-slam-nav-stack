#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

python3 - <<'PY'
import shutil
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path.cwd()
RVIZ_PATH = ROOT / 'src/slam_nav_piper_bringup/config/piper_visualization.rviz'
LAUNCH_PATH = ROOT / 'src/slam_nav_piper_bringup/launch/piper_visualization.launch.py'
failures = []


def ok(message):
    print(f'[Piper Viz] OK   {message}')


def fail(message):
    failures.append(message)
    print(f'[Piper Viz] FAIL {message}')


def run(command):
    result = subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    if result.returncode != 0:
        fail(f'命令失败: {" ".join(command)}\n{result.stdout[-2000:]}')
    return result.stdout


def launch_default_value(show_args, name):
    lines = show_args.splitlines()
    for index, line in enumerate(lines):
        if line.strip() == f"'{name}':":
            block = '\n'.join(lines[index:index + 8])
            for block_line in block.splitlines():
                stripped = block_line.strip()
                if stripped.startswith("(default: '") and stripped.endswith("')"):
                    return stripped[len("(default: '"):-2]
            return None
    return None


def launch_default(show_args, name, expected):
    value = launch_default_value(show_args, name)
    if value is None:
        fail(f'launch --show-args 未列出参数 {name}')
    elif value == expected:
        ok(f'launch 参数 {name} 默认值为 {expected}')
    else:
        fail(f'launch 参数 {name} 默认值不是 {expected}，实际为 {value}')


def collect_displays(config):
    displays = config.get('Visualization Manager', {}).get('Displays', [])
    return displays if isinstance(displays, list) else []


def display_topics(displays):
    topics = []
    for display in displays:
        if not isinstance(display, dict):
            continue
        topic = display.get('Topic')
        if isinstance(topic, dict) and isinstance(topic.get('Value'), str):
            topics.append(topic['Value'])
        description_topic = display.get('Description Topic')
        if isinstance(description_topic, dict) and isinstance(description_topic.get('Value'), str):
            topics.append(description_topic['Value'])
    return topics


def display_names(displays):
    return {display.get('Name') for display in displays if isinstance(display, dict)}


def display_classes(displays):
    return {display.get('Class') for display in displays if isinstance(display, dict)}


if shutil.which('rviz2'):
    ok('rviz2 可执行文件存在，图形可视化入口可启动。')
else:
    fail('未找到 rviz2 可执行文件，无法打开 Piper RViz 可视化。')

if RVIZ_PATH.exists():
    ok(f'RViz 配置存在: {RVIZ_PATH}')
else:
    fail(f'RViz 配置不存在: {RVIZ_PATH}')

if LAUNCH_PATH.exists():
    ok(f'可视化 launch 存在: {LAUNCH_PATH}')
else:
    fail(f'可视化 launch 不存在: {LAUNCH_PATH}')

show_args = run(['ros2', 'launch', 'slam_nav_piper_bringup', 'piper_visualization.launch.py', '--show-args'])
launch_default(show_args, 'arm_model', 'official')
launch_default(show_args, 'start_runtime', 'true')
launch_default(show_args, 'start_moveit_plan', 'false')
launch_default(show_args, 'use_sim_time', 'true')
rviz_default = launch_default_value(show_args, 'rviz_config')
if rviz_default is None:
    fail('launch --show-args 未列出 rviz_config 默认配置路径。')
else:
    default_rviz_path = Path(rviz_default)
    if default_rviz_path.exists():
        ok(f'launch 默认 RViz 配置存在: {default_rviz_path}')
        if RVIZ_PATH.exists() and default_rviz_path.read_text(encoding='utf-8') == RVIZ_PATH.read_text(encoding='utf-8'):
            ok('launch 默认 RViz 配置与源码配置一致。')
        else:
            fail('launch 默认 RViz 配置与源码配置不一致，请重新 colcon build 或检查安装目录。')
    else:
        fail(f'launch 默认 RViz 配置不存在: {default_rviz_path}')

launch_text = LAUNCH_PATH.read_text(encoding='utf-8')
required_launch_tokens = [
    'piper_sim.launch.py',
    'piper_project_moveit_plan.launch.py',
    "'allow_trajectory_execution': 'false'",
    "'joint_states_topic': '/piper/joint_states'",
]
for token in required_launch_tokens:
    if token in launch_text:
        ok(f'可视化 launch 保留 {token}')
    else:
        fail(f'可视化 launch 缺少 {token}')

config = yaml.safe_load(RVIZ_PATH.read_text(encoding='utf-8')) if RVIZ_PATH.exists() else {}
displays = collect_displays(config or {})
names = display_names(displays)
classes = display_classes(displays)
topics = set(display_topics(displays))

required_names = {
    'RobotModel',
    'TF',
    'Piper Color Image',
    'Piper Debug Image',
    'Target Pose',
    'Grasp Candidate Markers',
}
missing_names = sorted(required_names - names)
if missing_names:
    fail('RViz 配置缺少显示项: ' + ', '.join(missing_names))
else:
    ok('RViz 配置包含 RobotModel/TF/图像/目标位姿/抓取候选显示项。')

required_classes = {
    'rviz_default_plugins/RobotModel',
    'rviz_default_plugins/TF',
    'rviz_default_plugins/Image',
    'rviz_default_plugins/Pose',
    'rviz_default_plugins/MarkerArray',
}
missing_classes = sorted(required_classes - classes)
if missing_classes:
    fail('RViz 配置缺少显示插件: ' + ', '.join(missing_classes))
else:
    ok('RViz 配置包含所需显示插件。')

required_topics = {
    '/piper/robot_description',
    '/piper/arm_camera/color/image_raw',
    '/piper/perception/debug_image',
    '/piper/perception/target_pose',
    '/piper/visualization/grasp_candidates',
}
missing_topics = sorted(required_topics - topics)
if missing_topics:
    fail('RViz 配置缺少订阅话题: ' + ', '.join(missing_topics))
else:
    ok('RViz 配置订阅 Piper 机械臂可视化所需话题。')

fixed_frame = (config or {}).get('Visualization Manager', {}).get('Global Options', {}).get('Fixed Frame')
if fixed_frame == 'base_link':
    ok('RViz Fixed Frame 使用底盘 base_link，匹配独立 Piper 仿真 TF 根。')
else:
    fail(f'RViz Fixed Frame 应为 base_link，实际为 {fixed_frame}')

full_text = RVIZ_PATH.read_text(encoding='utf-8') if RVIZ_PATH.exists() else ''
for forbidden in ['/nav_camera', '/goal_pose', '/initialpose', 'costmap', 'nav2']:
    if forbidden.lower() in full_text.lower():
        fail(f'RViz 配置不应引用 {forbidden}')
    else:
        ok(f'RViz 配置未引用 {forbidden}')

bad_topics = [
    topic
    for topic in topics
    if topic.startswith('/') and not topic.startswith('/piper/')
]
if bad_topics:
    fail('RViz 配置中仍有非 Piper 绝对话题: ' + ', '.join(sorted(set(bad_topics))))
else:
    ok('RViz 配置中的绝对话题全部位于 /piper 命名空间。')

if failures:
    print()
    print('[Piper Viz] 可视化配置烟测失败。')
    sys.exit(2)

print()
print('[Piper Viz] 可视化配置烟测通过：可打开 RViz 观察 Piper，但不会提供 Nav2 目标/初始位姿工具。')
PY
