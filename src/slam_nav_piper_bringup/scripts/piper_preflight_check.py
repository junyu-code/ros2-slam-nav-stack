#!/usr/bin/env python3

import argparse
import importlib.util
import json
import sys
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_prefix


REQUIRED_PROJECT_PACKAGES = [
    'slam_nav_piper_interfaces',
    'slam_nav_piper_description',
    'slam_nav_piper_perception',
    'slam_nav_piper_control',
    'slam_nav_piper_moveit_config',
    'slam_nav_piper_manipulation',
    'slam_nav_piper_calibration',
    'slam_nav_piper_bringup',
    'slam_nav_piper_learning',
]

REQUIRED_ROS_CONTROL_PACKAGES = [
    'controller_manager',
    'joint_state_broadcaster',
    'joint_trajectory_controller',
    'gripper_controllers',
    'trajectory_msgs',
]

REQUIRED_MOVEIT_PACKAGES = [
    'moveit_ros_planning_interface',
    'moveit_ros_move_group',
    'moveit_core',
    'moveit_kinematics',
    'moveit_planners_ompl',
    'moveit_simple_controller_manager',
]

OPTIONAL_MOVEIT_DEMO_PACKAGES = [
    'moveit_configs_utils',
    'moveit_ros_visualization',
]

OPTIONAL_OFFICIAL_PACKAGES = [
    'piper_description',
    'piper_moveit_config_v4',
    'piper_moveit_config_v5',
]

OFFICIAL_KEY_FILES = [
    'piper_description/package.xml',
    'piper_description/urdf/piper_description.xacro',
    'piper_description/meshes/base_link.STL',
    'piper_description/meshes/link1.STL',
    'piper_description/meshes/link2.STL',
    'piper_description/meshes/link3.STL',
    'piper_description/meshes/link4.STL',
    'piper_description/meshes/link5.STL',
    'piper_description/meshes/link6.STL',
    'piper_description/meshes/link7.STL',
    'piper_description/meshes/link8.STL',
    'piper_moveit_config_v4/package.xml',
    'piper_moveit_config_v4/config/piper.srdf',
    'piper_moveit_config_v4/launch/demo.launch.py',
    'piper_moveit_config_v5/package.xml',
    'piper_moveit_config_v5/config/piper.srdf',
    'piper_moveit_config_v5/launch/demo.launch.py',
]


def check_ros_package(package_name):
    try:
        return True, get_package_prefix(package_name)
    except PackageNotFoundError:
        return False, ''


def print_group(title, package_names, required=True):
    ok = True
    print(f'\n[{title}]')
    for package_name in package_names:
        found, prefix = check_ros_package(package_name)
        if found:
            print(f'  OK      {package_name}: {prefix}')
        else:
            marker = 'MISSING' if required else 'OPTIONAL'
            print(f'  {marker} {package_name}')
            ok = ok and not required
    return ok


def check_python_module(module_name, required=True):
    found = importlib.util.find_spec(module_name) is not None
    marker = 'OK' if found else ('MISSING' if required else 'OPTIONAL')
    print(f'  {marker:<7} python:{module_name}')
    return found or not required


def find_workspace_root():
    current = Path.cwd().resolve()
    for candidate in [current, *current.parents]:
        if (candidate / 'src').exists() and (candidate / '.git').exists():
            return candidate
    return current


def print_external_download_status():
    workspace_root = find_workspace_root()
    piper_dir = workspace_root / 'external' / 'agilex' / 'agilex_open_class' / 'piper'
    print('\n[AgileX open class 下载目录]')
    if not piper_dir.exists():
        print(f'  OPTIONAL 未找到 {piper_dir}')
        return

    files = [path for path in piper_dir.rglob('*') if path.is_file()]
    total_size = sum(path.stat().st_size for path in files)
    print(f'  路径: {piper_dir}')
    print(f'  已有文件: {len(files)}, 大小: {total_size / 1024 / 1024:.1f} MiB')
    manifest_path = piper_dir.parent / '.piper_open_class_manifest.json'
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
            print(
                '  manifest: '
                f'complete={manifest.get("complete")}, '
                f'existing={manifest.get("existing_count")}, '
                f'missing={manifest.get("missing_count")}'
            )
            if manifest.get('failed_path'):
                print(f'  last_failed_path: {manifest.get("failed_path")}')
        except Exception as exc:
            print(f'  manifest 读取失败: {exc}')
    for rel_path in OFFICIAL_KEY_FILES:
        path = piper_dir / rel_path
        marker = 'OK' if path.exists() else 'MISSING'
        print(f'  {marker:<7} {rel_path}')


def main():
    parser = argparse.ArgumentParser(description='Piper 移动操作依赖预检。')
    parser.add_argument(
        '--require-official',
        action='store_true',
        help='要求 AgileX 官方 piper_description / MoveIt2 配置包必须存在。',
    )
    args = parser.parse_args()

    all_ok = True
    all_ok &= print_group('项目侧 Piper 包', REQUIRED_PROJECT_PACKAGES, required=True)
    all_ok &= print_group('MoveIt2 二进制包', REQUIRED_MOVEIT_PACKAGES, required=True)
    print_group('MoveIt2 官方 demo/RViz 可选包', OPTIONAL_MOVEIT_DEMO_PACKAGES, required=False)
    all_ok &= print_group('ros2_control / controller 包', REQUIRED_ROS_CONTROL_PACKAGES, required=True)
    all_ok &= print_group('AgileX 官方 Piper 包', OPTIONAL_OFFICIAL_PACKAGES, required=args.require_official)
    print_external_download_status()

    print('\n[Python 模块]')
    all_ok &= check_python_module('yaml', required=True)

    print('\n[边界提醒]')
    print('  Piper 默认不接入 task1，不修改 /nav_camera，也不作为 Nav2 costmap 默认观测源。')
    print('  官方包缺失时仍可运行项目侧 placeholder/fake Piper 冒烟链路。')
    print('  MoveIt2 plan-only 需要 OMPL 与 simple controller manager：')
    print('    sudo apt-get install ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager')
    print('    或运行 ./run.sh setup-piper-moveit 使用 external/ 下的 Piper 专用本地 overlay。')
    print('  官方 MoveIt2/RViz demo 可选：sudo apt-get install ros-humble-moveit-configs-utils ros-humble-moveit-ros-visualization')
    print('  实机前必须验证急停、失能、home、限速和工作空间限制。')

    if all_ok:
        print('\nPiper 预检通过。')
        return 0

    print('\nPiper 预检未通过：请先补齐上面标记为 MISSING 的必需项。')
    return 2


if __name__ == '__main__':
    sys.exit(main())
