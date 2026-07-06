#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

python3 - <<'PY'
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path.cwd()
failures = []


def fail(message):
    failures.append(message)
    print(f'[Piper Boundary] FAIL {message}')


def ok(message):
    print(f'[Piper Boundary] OK   {message}')


def run(command):
    result = subprocess.run(
        command,
        cwd=ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if result.returncode != 0:
        fail(f'命令失败: {" ".join(command)}\n{result.stdout[-2000:]}')
    return result.stdout


def launch_default(show_args, name, expected):
    lines = show_args.splitlines()
    for index, line in enumerate(lines):
        if line.strip() == f"'{name}':":
            block = '\n'.join(lines[index:index + 8])
            if f"(default: '{expected}')" in block:
                ok(f'{name} 默认值为 {expected}')
            else:
                fail(f'{name} 默认值不是 {expected}，实际片段:\n{block}')
            return
    fail(f'未在 launch --show-args 中找到参数 {name}')


def text_files(paths):
    suffixes = {'.py', '.yaml', '.yml', '.xml', '.rviz', '.sh', '.txt'}
    for root in paths:
        if root.is_file():
            candidates = [root]
        elif root.exists():
            candidates = root.rglob('*')
        else:
            continue
        for path in candidates:
            if path.is_file() and path.suffix in suffixes and '__pycache__' not in path.parts:
                yield path


def assert_no_token(paths, token, label):
    offenders = []
    for path in text_files(paths):
        text = path.read_text(encoding='utf-8', errors='ignore')
        if token.lower() in text.lower():
            offenders.append(path)
    if offenders:
        fail(f'{label} 不应包含 {token}: ' + ', '.join(str(path) for path in offenders[:12]))
    else:
        ok(f'{label} 未引用 {token}')


def flatten_values(value):
    if isinstance(value, dict):
        for item in value.values():
            yield from flatten_values(item)
    elif isinstance(value, list):
        for item in value:
            yield from flatten_values(item)
    else:
        yield value


show_args = run(['ros2', 'launch', 'slam_nav_simulation', 'simulation.launch.py', '--show-args'])
launch_default(show_args, 'enable_piper_arm', 'false')
launch_default(show_args, 'enable_nav_rgbd_camera', 'false')
launch_default(show_args, 'piper_arm_model', 'official')

builder = ROOT / 'src/slam_nav_piper_description/scripts/piper_description_builder.py'
base_xacro = ROOT / 'src/slam_nav_simulation/urdf/mobile_robot.xacro'
default_urdf = run([
    'python3',
    str(builder),
    '--base-xacro',
    str(base_xacro),
    '--enable-piper-arm',
    'false',
    '--arm-model',
    'official',
])
if any(token in default_urdf for token in ('piper_base_link', 'piper_arm_camera_link', 'piper_joint1')):
    fail('默认 enable_piper_arm=false 时 robot_description 不应包含 Piper 链路。')
else:
    ok('默认 enable_piper_arm=false 时 robot_description 不包含 Piper 链路。')

official_urdf = run([
    'python3',
    str(builder),
    '--base-xacro',
    str(base_xacro),
    '--enable-piper-arm',
    'true',
    '--arm-model',
    'official',
])
required_tokens = ('piper_base_link', 'piper_joint1', 'piper_joint8', 'piper_arm_camera_optical_frame')
if all(token in official_urdf for token in required_tokens) and 'piper_joint1_placeholder' not in official_urdf:
    ok('显式 enable_piper_arm=true 时使用官方 Piper piper_* 适配链。')
else:
    fail('显式 enable_piper_arm=true 时未得到完整官方 Piper 适配链，或仍含占位关节。')

assert_no_token(
    [
        ROOT / 'src/slam_nav_bringup/config',
        ROOT / 'src/slam_nav_bringup/launch',
        ROOT / 'src/slam_nav_bringup/scripts',
    ],
    '/piper',
    'Nav2/task1 bringup 配置',
)

assert_no_token(
    [
        ROOT / 'scripts/start_mapping.sh',
        ROOT / 'scripts/start_navigation.sh',
        ROOT / 'scripts/start_navigation_3d.sh',
        ROOT / 'scripts/start_simulation.sh',
        ROOT / 'scripts/start_simulation_static.sh',
        ROOT / 'scripts/start_simulation_dynamic.sh',
        ROOT / 'scripts/start_simulation_dynamic_rgbd.sh',
    ],
    '/piper',
    'task1/default 顶层脚本',
)

perception_config = yaml.safe_load(
    (ROOT / 'src/slam_nav_piper_perception/config/perception.yaml').read_text(encoding='utf-8')
)
bad_values = [
    value
    for value in flatten_values(perception_config)
    if isinstance(value, str) and value.startswith('/nav_camera')
]
if bad_values:
    fail('Piper perception 配置中不应把机械臂相机指向 /nav_camera: ' + ', '.join(bad_values))
else:
    ok('Piper perception 配置只使用 /piper/arm_camera/* 输入。')

if failures:
    print()
    print('[Piper Boundary] 边界检查失败。')
    sys.exit(2)

print()
print('[Piper Boundary] 边界检查通过。')
PY
