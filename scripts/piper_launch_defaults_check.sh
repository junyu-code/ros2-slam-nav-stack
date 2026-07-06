#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

python3 - <<'PY'
import re
import subprocess
import sys
from pathlib import Path


ROOT = Path.cwd()
failures = []


def fail(message):
    failures.append(message)
    print(f'[Piper Launch Defaults] FAIL {message}')


def ok(message):
    print(f'[Piper Launch Defaults] OK   {message}')


def show_args(package, launch_file):
    result = subprocess.run(
        ['ros2', 'launch', package, launch_file, '--show-args'],
        cwd=ROOT,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if result.returncode != 0:
        fail(f'{package}/{launch_file} 参数展开失败:\n{result.stdout[-2000:]}')
        return ''
    return result.stdout


def parse_defaults(text):
    defaults = {}
    current_name = None
    for line in text.splitlines():
        argument_match = re.match(r"\s+'([^']+)':\s*$", line)
        if argument_match:
            current_name = argument_match.group(1)
            continue
        default_match = re.search(r"\(default: '([^']*)'\)", line)
        if current_name and default_match and current_name not in defaults:
            defaults[current_name] = default_match.group(1)
    return defaults


def expect_default(defaults, package, launch_file, name, expected):
    actual = defaults.get(name)
    if actual == expected:
        ok(f'{package}/{launch_file}: {name} 默认值为 {expected}')
    elif actual is None:
        fail(f'{package}/{launch_file}: 未找到 launch 参数 {name}')
    else:
        fail(f'{package}/{launch_file}: {name} 默认值应为 {expected}，实际为 {actual}')


def expect_source_contains(relative_path, token, label):
    path = ROOT / relative_path
    if not path.exists():
        fail(f'{label}: 文件不存在 {relative_path}')
        return
    text = path.read_text(encoding='utf-8')
    if token in text:
        ok(f'{label}: 包含安全边界 {token}')
    else:
        fail(f'{label}: 缺少安全边界 {token}')


checks = {
    ('slam_nav_piper_bringup', 'piper_real.launch.py'): {
        'use_sim_time': 'false',
        'backend': 'moveit',
        'arm_model': 'official',
        'real_backend_connected': 'false',
        'fake_camera': 'false',
        'auto_enable': 'false',
        'initial_owner': 'disabled',
        'enable_piper_gazebo_camera': 'false',
        'publish_joint_states': 'false',
    },
    ('slam_nav_piper_bringup', 'piper_sim.launch.py'): {
        'use_sim_time': 'true',
        'arm_model': 'official',
        'publish_joint_states': 'true',
        'require_moveit_plan_before_fake_execution': 'false',
        'moveit_plan_service': '/piper/plan_kinematic_path',
        'moveit_plan_service_timeout_s': '10.0',
    },
    ('slam_nav_piper_bringup', 'piper_mobile_manipulation.launch.py'): {
        'use_sim_time': 'true',
        'arm_model': 'official',
        'start_description': 'false',
        'publish_joint_states': 'true',
        'fake_camera': 'false',
        'fake_execution': 'true',
        'real_backend_connected': 'false',
        'publish_base_stop': 'false',
        'use_ranked_grasp_candidates': 'false',
        'require_moveit_plan_before_fake_execution': 'false',
        'moveit_plan_service': '/piper/plan_kinematic_path',
    },
    ('slam_nav_piper_bringup', 'piper_visualization.launch.py'): {
        'use_sim_time': 'true',
        'arm_model': 'official',
        'start_runtime': 'true',
        'start_moveit_plan': 'false',
    },
    ('slam_nav_piper_moveit_config', 'piper_project_moveit_plan.launch.py'): {
        'description_mode': 'standalone',
        'publish_robot_state': 'true',
        'start_joint_state_publisher': 'true',
        'allow_trajectory_execution': 'false',
        'joint_states_topic': '/piper/joint_states',
        'piper_tcp_parent_link': 'piper_link6',
    },
    ('slam_nav_piper_learning', 'piper_learning.launch.py'): {
        'enable_learning': 'false',
        'policy_backend': 'disabled',
    },
}

for (package, launch_file), expected_defaults in checks.items():
    defaults = parse_defaults(show_args(package, launch_file))
    for name, expected in expected_defaults.items():
        expect_default(defaults, package, launch_file, name, expected)

# show-args 会列出被 include launch 的原始默认值；这里补充检查父 launch 传入的安全覆盖。
expect_source_contains(
    'src/slam_nav_piper_bringup/launch/piper_real.launch.py',
    "'fake_execution': 'false'",
    'piper_real 实机入口',
)
expect_source_contains(
    'src/slam_nav_piper_bringup/launch/piper_real.launch.py',
    "'publish_base_stop': 'false'",
    'piper_real 实机入口',
)
expect_source_contains(
    'src/slam_nav_piper_bringup/launch/piper_visualization.launch.py',
    "'allow_trajectory_execution': 'false'",
    'piper_visualization 可视化入口',
)

if failures:
    print()
    print('[Piper Launch Defaults] launch 默认值检查失败。')
    sys.exit(2)

print()
print('[Piper Launch Defaults] launch 默认值检查通过。')
PY
