#!/usr/bin/env python3

import argparse
import math
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


ARM_JOINTS = [f'piper_joint{index}' for index in range(1, 7)]
GRIPPER_JOINTS = [f'piper_joint{index}' for index in range(7, 9)]
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS
OFFICIAL_ARM_GROUP = 'arm'
OFFICIAL_GRIPPER_GROUP = 'gripper'
PROJECT_ARM_GROUP = 'piper_arm'
PROJECT_GRIPPER_GROUP = 'piper_gripper'
PROJECT_ARM_CONTROLLER = 'piper_arm_controller'
PROJECT_GRIPPER_CONTROLLER = 'piper_gripper_controller'


def find_workspace_root():
    current = Path.cwd().resolve()
    for candidate in [current, *current.parents]:
        if (candidate / 'src' / 'slam_nav_piper_moveit_config').exists():
            return candidate
    return None


def source_package_path(package_name):
    workspace_root = find_workspace_root()
    if workspace_root is None:
        return None
    candidate = workspace_root / 'src' / package_name
    return candidate if candidate.exists() else None


def package_share_path(package_name):
    return Path(get_package_share_directory(package_name))


def config_dir_from_args(args):
    if args.config_dir:
        return Path(args.config_dir).resolve()

    source_path = source_package_path('slam_nav_piper_moveit_config')
    if source_path is not None:
        return source_path / 'config'

    return package_share_path('slam_nav_piper_moveit_config') / 'config'


def description_builder_path():
    source_path = source_package_path('slam_nav_piper_description')
    if source_path is not None:
        candidate = source_path / 'scripts' / 'piper_description_builder.py'
        if candidate.exists():
            return candidate
    return package_share_path('slam_nav_piper_description') / 'scripts' / 'piper_description_builder.py'


def read_yaml(path):
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def read_xml(path):
    return ET.fromstring(path.read_text(encoding='utf-8'))


def map_official_name(name):
    if name == 'base_link':
        return 'piper_base_link'
    if re.fullmatch(r'link\d+', name):
        return f'piper_{name}'
    if re.fullmatch(r'joint\d+', name):
        return f'piper_{name}'
    return name


def map_official_group(name):
    if name == OFFICIAL_ARM_GROUP:
        return PROJECT_ARM_GROUP
    if name == OFFICIAL_GRIPPER_GROUP:
        return PROJECT_GRIPPER_GROUP
    return name


def map_official_controller(name):
    if name == 'arm_controller':
        return PROJECT_ARM_CONTROLLER
    if name == 'gripper_controller':
        return PROJECT_GRIPPER_CONTROLLER
    return name


def parse_urdf(xml_text):
    root = ET.fromstring(xml_text)
    links = {element.attrib['name'] for element in root.findall('link') if 'name' in element.attrib}
    joints = {element.attrib['name'] for element in root.findall('joint') if 'name' in element.attrib}
    return links, joints


def parse_srdf_groups(root):
    groups = {}
    chains = {}
    for group in root.findall('group'):
        name = group.attrib.get('name', '')
        groups[name] = [joint.attrib['name'] for joint in group.findall('joint') if 'name' in joint.attrib]
        chain = group.find('chain')
        if chain is not None:
            chains[name] = {
                'base_link': chain.attrib.get('base_link', ''),
                'tip_link': chain.attrib.get('tip_link', ''),
            }
    return groups, chains


def parse_srdf_group_states(root, mapper=None):
    states = {}
    for state in root.findall('group_state'):
        name = state.attrib.get('name', '')
        group = state.attrib.get('group', '')
        if mapper is not None:
            group = map_official_group(group)
        joint_values = {}
        for joint in state.findall('joint'):
            joint_name = joint.attrib.get('name', '')
            if mapper is not None:
                joint_name = mapper(joint_name)
            joint_values[joint_name] = str(joint.attrib.get('value', ''))
        states[(name, group)] = joint_values
    return states


def parse_srdf_disable_pairs(root, mapper=None):
    pairs = set()
    for element in root.findall('disable_collisions'):
        link1 = element.attrib.get('link1', '')
        link2 = element.attrib.get('link2', '')
        if mapper is not None:
            link1 = mapper(link1)
            link2 = mapper(link2)
        pairs.add(tuple(sorted((link1, link2))))
    return pairs


def render_project_official_urdf(args):
    builder = description_builder_path()
    command = [
        'python3',
        str(builder),
        '--arm-model',
        'official',
        '--official-description-package',
        args.official_description_package,
        '--official-description-xacro',
        args.official_description_xacro,
        '--tcp-parent-link',
        args.tcp_parent_link,
    ]
    result = subprocess.run(command, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return result.stdout


class Audit:
    def __init__(self):
        self.failures = []

    def ok(self, message):
        print(f'[Piper MoveIt2 Config] OK   {message}')

    def fail(self, message):
        print(f'[Piper MoveIt2 Config] FAIL {message}')
        self.failures.append(message)

    def expect(self, condition, ok_message, fail_message):
        if condition:
            self.ok(ok_message)
        else:
            self.fail(fail_message)


def check_project_urdf(audit, args):
    try:
        xml_text = render_project_official_urdf(args)
    except Exception as exc:
        audit.fail(f'官方 Piper URDF 适配链渲染失败: {exc}')
        return set(), set()

    if 'placeholder' in xml_text:
        audit.fail('官方 Piper URDF 适配链中不应出现 placeholder。')

    try:
        links, joints = parse_urdf(xml_text)
    except Exception as exc:
        audit.fail(f'官方 Piper URDF 适配链 XML 解析失败: {exc}')
        return set(), set()

    required_links = {
        'piper_base_link',
        'piper_link1',
        'piper_link2',
        'piper_link3',
        'piper_link4',
        'piper_link5',
        'piper_link6',
        'piper_link7',
        'piper_link8',
        'piper_tcp',
        'piper_arm_camera_link',
        'piper_arm_camera_optical_frame',
    }
    missing_links = sorted(required_links - links)
    missing_joints = sorted(set(ALL_JOINTS) - joints)
    audit.expect(
        not missing_links,
        '官方 URDF 适配链包含 Piper 关节链、TCP 和腕部相机 frame。',
        '官方 URDF 适配链缺少 link/frame: ' + ', '.join(missing_links),
    )
    audit.expect(
        not missing_joints,
        '官方 URDF 适配链包含 piper_joint1 到 piper_joint8。',
        '官方 URDF 适配链缺少 joint: ' + ', '.join(missing_joints),
    )
    return links, joints


def check_project_srdf(audit, config_dir, urdf_links, urdf_joints):
    srdf_path = config_dir / 'piper.srdf'
    root = read_xml(srdf_path)
    groups, chains = parse_srdf_groups(root)
    group_states = parse_srdf_group_states(root)
    disable_pairs = parse_srdf_disable_pairs(root)

    audit.expect(
        groups.get(PROJECT_ARM_GROUP) == ARM_JOINTS,
        'SRDF piper_arm 使用 piper_joint1 到 piper_joint6。',
        f'SRDF piper_arm 关节不匹配: {groups.get(PROJECT_ARM_GROUP)}',
    )
    audit.expect(
        groups.get(PROJECT_GRIPPER_GROUP) == GRIPPER_JOINTS,
        'SRDF piper_gripper 使用 piper_joint7 和 piper_joint8。',
        f'SRDF piper_gripper 关节不匹配: {groups.get(PROJECT_GRIPPER_GROUP)}',
    )
    audit.expect(
        chains.get(PROJECT_ARM_GROUP) == {'base_link': 'piper_base_link', 'tip_link': 'piper_tcp'},
        'SRDF piper_arm chain 使用 piper_base_link -> piper_tcp。',
        f'SRDF piper_arm chain 不匹配: {chains.get(PROJECT_ARM_GROUP)}',
    )

    srdf_joint_refs = set()
    srdf_link_refs = set()
    for group_joints in groups.values():
        srdf_joint_refs.update(group_joints)
    for values in group_states.values():
        srdf_joint_refs.update(values.keys())
    for chain in chains.values():
        srdf_link_refs.add(chain.get('base_link', ''))
        srdf_link_refs.add(chain.get('tip_link', ''))
    for pair in disable_pairs:
        srdf_link_refs.update(pair)

    missing_joint_refs = sorted(srdf_joint_refs - urdf_joints)
    missing_link_refs = sorted(srdf_link_refs - urdf_links)
    audit.expect(
        not missing_joint_refs,
        'SRDF 里的 joint 引用都能在官方适配 URDF 中找到。',
        'SRDF 引用了 URDF 中不存在的 joint: ' + ', '.join(missing_joint_refs),
    )
    audit.expect(
        not missing_link_refs,
        'SRDF 里的 link 引用都能在官方适配 URDF 中找到。',
        'SRDF 引用了 URDF 中不存在的 link: ' + ', '.join(missing_link_refs),
    )

    unprefixed_joints = sorted(name for name in srdf_joint_refs if re.fullmatch(r'joint\d+', name))
    unprefixed_links = sorted(name for name in srdf_link_refs if name == 'base_link' or re.fullmatch(r'link\d+', name))
    audit.expect(
        not unprefixed_joints and not unprefixed_links,
        'SRDF 不再引用官方原生 base_link/link*/joint* 名称。',
        f'SRDF 仍有未适配名称: joints={unprefixed_joints}, links={unprefixed_links}',
    )
    return root


def controller_joints_from_ros2_control(config, controller_name):
    return (
        config.get(controller_name, {})
        .get('ros__parameters', {})
        .get('joints', [])
    )


def check_yaml_configs(audit, config_dir):
    joint_limits = read_yaml(config_dir / 'joint_limits.yaml')
    ros2_controllers = read_yaml(config_dir / 'ros2_controllers.yaml')
    moveit_controllers = read_yaml(config_dir / 'moveit_controllers.yaml')
    initial_positions = read_yaml(config_dir / 'initial_positions.yaml')
    kinematics = read_yaml(config_dir / 'kinematics.yaml')

    limit_joints = sorted((joint_limits.get('joint_limits') or {}).keys())
    audit.expect(
        limit_joints == ALL_JOINTS,
        'joint_limits.yaml 覆盖 piper_joint1 到 piper_joint8。',
        f'joint_limits.yaml 关节集合不匹配: {limit_joints}',
    )
    audit.expect(
        float(joint_limits.get('default_velocity_scaling_factor', 1.0)) <= 0.10
        and float(joint_limits.get('default_acceleration_scaling_factor', 1.0)) <= 0.10,
        'joint_limits.yaml 默认速度/加速度比例保持 0.10 以内。',
        'joint_limits.yaml 默认速度/加速度比例超过 0.10。',
    )

    arm_ros2_joints = controller_joints_from_ros2_control(ros2_controllers, PROJECT_ARM_CONTROLLER)
    gripper_ros2_joints = controller_joints_from_ros2_control(ros2_controllers, PROJECT_GRIPPER_CONTROLLER)
    audit.expect(
        arm_ros2_joints == ARM_JOINTS and gripper_ros2_joints == GRIPPER_JOINTS,
        'ros2_controllers.yaml 控制器关节集合与 Piper 机械臂/夹爪分组一致。',
        f'ros2_controllers.yaml 关节不匹配: arm={arm_ros2_joints}, gripper={gripper_ros2_joints}',
    )

    moveit_simple = moveit_controllers.get('moveit_simple_controller_manager', {})
    controller_names = moveit_simple.get('controller_names', [])
    moveit_arm_joints = moveit_simple.get(PROJECT_ARM_CONTROLLER, {}).get('joints', [])
    moveit_gripper_joints = moveit_simple.get(PROJECT_GRIPPER_CONTROLLER, {}).get('joints', [])
    audit.expect(
        controller_names == [PROJECT_ARM_CONTROLLER, PROJECT_GRIPPER_CONTROLLER]
        and moveit_arm_joints == ARM_JOINTS
        and moveit_gripper_joints == GRIPPER_JOINTS,
        'moveit_controllers.yaml 只暴露项目侧 Piper 控制器。',
        (
            'moveit_controllers.yaml 控制器不匹配: '
            f'names={controller_names}, arm={moveit_arm_joints}, gripper={moveit_gripper_joints}'
        ),
    )

    positions = initial_positions.get('initial_positions', {})
    audit.expect(
        sorted(positions.keys()) == ALL_JOINTS,
        'initial_positions.yaml 覆盖 piper_joint1 到 piper_joint8。',
        f'initial_positions.yaml 关节集合不匹配: {sorted(positions.keys())}',
    )
    audit.expect(
        PROJECT_ARM_GROUP in kinematics and OFFICIAL_ARM_GROUP not in kinematics,
        'kinematics.yaml 使用 piper_arm 规划组名称。',
        f'kinematics.yaml 规划组不匹配: {sorted(kinematics.keys())}',
    )
    return {
        'joint_limits': joint_limits,
        'ros2_controllers': ros2_controllers,
        'moveit_controllers': moveit_controllers,
        'initial_positions': initial_positions,
        'kinematics': kinematics,
    }


def check_fake_joint_publisher(audit):
    source_path = source_package_path('slam_nav_piper_moveit_config')
    if source_path is None:
        audit.ok('未找到源码目录，跳过 fake joint publisher 源码文本检查。')
        return

    script_path = source_path / 'scripts' / 'piper_fake_joint_state_publisher.py'
    text = script_path.read_text(encoding='utf-8')
    missing = [joint for joint in ALL_JOINTS if joint not in text]
    audit.expect(
        not missing,
        'fake joint publisher 默认发布 piper_joint1 到 piper_joint8。',
        'fake joint publisher 缺少默认关节: ' + ', '.join(missing),
    )


def check_official_srdf_mapping(audit, config_dir, project_srdf_root, official_package):
    try:
        official_config_dir = package_share_path(official_package) / 'config'
    except PackageNotFoundError:
        audit.fail(f'未找到 AgileX 官方 MoveIt2 配置包 {official_package}。')
        return

    official_srdf_path = official_config_dir / 'piper.srdf'
    if not official_srdf_path.exists():
        audit.fail(f'官方 SRDF 不存在: {official_srdf_path}')
        return

    official_root = read_xml(official_srdf_path)
    official_groups, _ = parse_srdf_groups(official_root)
    project_groups, _ = parse_srdf_groups(project_srdf_root)

    mapped_arm = [map_official_name(name) for name in official_groups.get(OFFICIAL_ARM_GROUP, [])]
    mapped_gripper = [map_official_name(name) for name in official_groups.get(OFFICIAL_GRIPPER_GROUP, [])]
    audit.expect(
        project_groups.get(PROJECT_ARM_GROUP) == mapped_arm
        and project_groups.get(PROJECT_GRIPPER_GROUP) == mapped_gripper,
        f'项目 SRDF 分组与 {official_package} 官方 arm/gripper 关节映射一致。',
        (
            f'项目 SRDF 分组与 {official_package} 不一致: '
            f'official_arm={mapped_arm}, project_arm={project_groups.get(PROJECT_ARM_GROUP)}, '
            f'official_gripper={mapped_gripper}, project_gripper={project_groups.get(PROJECT_GRIPPER_GROUP)}'
        ),
    )

    official_states = parse_srdf_group_states(official_root, mapper=map_official_name)
    project_states = parse_srdf_group_states(project_srdf_root)
    missing_or_changed_states = []
    for key, official_values in official_states.items():
        mapped_key = (key[0], map_official_group(key[1]))
        if project_states.get(mapped_key) != official_values:
            missing_or_changed_states.append(mapped_key)
    audit.expect(
        not missing_or_changed_states,
        f'项目 SRDF group_state 与 {official_package} 官方配置映射一致。',
        f'项目 SRDF group_state 与官方配置不一致: {missing_or_changed_states}',
    )

    official_pairs = parse_srdf_disable_pairs(official_root, mapper=map_official_name)
    project_pairs = parse_srdf_disable_pairs(project_srdf_root)
    missing_pairs = sorted(official_pairs - project_pairs)
    audit.expect(
        not missing_pairs,
        f'项目 SRDF 保留 {official_package} 官方禁碰撞对，并额外补充 TCP/相机。',
        f'项目 SRDF 缺少官方禁碰撞对: {missing_pairs}',
    )


def nearly_equal(value_a, value_b):
    try:
        return math.isclose(float(value_a), float(value_b), rel_tol=1e-6, abs_tol=1e-9)
    except Exception:
        return value_a == value_b


def check_official_yaml_mapping(audit, project_configs, official_package):
    try:
        official_config_dir = package_share_path(official_package) / 'config'
    except PackageNotFoundError:
        audit.fail(f'未找到 AgileX 官方 MoveIt2 配置包 {official_package}。')
        return

    official_limits = read_yaml(official_config_dir / 'joint_limits.yaml')
    official_moveit_controllers = read_yaml(official_config_dir / 'moveit_controllers.yaml')
    official_ros2_controllers = read_yaml(official_config_dir / 'ros2_controllers.yaml')

    project_limits = project_configs['joint_limits'].get('joint_limits', {})
    changed_limits = []
    for official_name, official_limit in (official_limits.get('joint_limits') or {}).items():
        project_name = map_official_name(official_name)
        project_limit = project_limits.get(project_name, {})
        for key in ('has_velocity_limits', 'max_velocity', 'has_acceleration_limits', 'max_acceleration'):
            if key not in project_limit or not nearly_equal(project_limit.get(key), official_limit.get(key)):
                changed_limits.append((project_name, key))
    audit.expect(
        not changed_limits,
        f'joint_limits.yaml 与 {official_package} 官方关节限制映射一致。',
        f'joint_limits.yaml 与官方限制不一致: {changed_limits}',
    )

    official_simple = official_moveit_controllers.get('moveit_simple_controller_manager', {})
    project_simple = project_configs['moveit_controllers'].get('moveit_simple_controller_manager', {})
    mapped_controller_names = [
        map_official_controller(name)
        for name in official_simple.get('controller_names', [])
    ]
    mapped_controller_joints = {}
    for official_controller in official_simple.get('controller_names', []):
        project_controller = map_official_controller(official_controller)
        mapped_controller_joints[project_controller] = [
            map_official_name(name)
            for name in official_simple.get(official_controller, {}).get('joints', [])
        ]
    controller_ok = project_simple.get('controller_names') == mapped_controller_names
    for controller_name, joints in mapped_controller_joints.items():
        controller_ok = controller_ok and project_simple.get(controller_name, {}).get('joints') == joints
    audit.expect(
        controller_ok,
        f'moveit_controllers.yaml 与 {official_package} 官方控制器映射一致。',
        (
            'moveit_controllers.yaml 与官方控制器不一致: '
            f'mapped_names={mapped_controller_names}, project_names={project_simple.get("controller_names")}'
        ),
    )

    official_arm_joints = controller_joints_from_ros2_control(official_ros2_controllers, 'arm_controller')
    official_gripper_joints = controller_joints_from_ros2_control(official_ros2_controllers, 'gripper_controller')
    project_arm_joints = controller_joints_from_ros2_control(project_configs['ros2_controllers'], PROJECT_ARM_CONTROLLER)
    project_gripper_joints = controller_joints_from_ros2_control(project_configs['ros2_controllers'], PROJECT_GRIPPER_CONTROLLER)
    audit.expect(
        project_arm_joints == [map_official_name(name) for name in official_arm_joints]
        and project_gripper_joints == [map_official_name(name) for name in official_gripper_joints],
        f'ros2_controllers.yaml 与 {official_package} 官方 ros2_control 关节映射一致。',
        (
            'ros2_controllers.yaml 与官方 ros2_control 配置不一致: '
            f'project_arm={project_arm_joints}, project_gripper={project_gripper_joints}'
        ),
    )


def main():
    parser = argparse.ArgumentParser(description='审计 Piper 项目侧 MoveIt2 配置与 AgileX 官方配置的映射一致性。')
    parser.add_argument('--config-dir', default='', help='可选：项目侧 MoveIt2 config 目录。')
    parser.add_argument('--official-moveit-package', default='piper_moveit_config_v5')
    parser.add_argument('--official-description-package', default='piper_description')
    parser.add_argument('--official-description-xacro', default='urdf/piper_description.xacro')
    parser.add_argument('--tcp-parent-link', default='piper_link6')
    parser.add_argument(
        '--allow-missing-official',
        action='store_true',
        help='官方包缺失时只检查项目侧文件；默认要求官方包存在。',
    )
    args = parser.parse_args()

    audit = Audit()
    config_dir = config_dir_from_args(args)
    print(f'[Piper MoveIt2 Config] 项目配置目录: {config_dir}')

    if not config_dir.exists():
        audit.fail(f'项目侧 MoveIt2 config 目录不存在: {config_dir}')
        return 2

    urdf_links, urdf_joints = check_project_urdf(audit, args)
    project_srdf_root = check_project_srdf(audit, config_dir, urdf_links, urdf_joints)
    project_configs = check_yaml_configs(audit, config_dir)
    check_fake_joint_publisher(audit)

    try:
        package_share_path(args.official_moveit_package)
    except PackageNotFoundError:
        if args.allow_missing_official:
            audit.ok(f'官方 MoveIt2 配置包 {args.official_moveit_package} 缺失，已按参数跳过官方映射对比。')
        else:
            audit.fail(f'未找到官方 MoveIt2 配置包 {args.official_moveit_package}。')
    else:
        check_official_srdf_mapping(audit, config_dir, project_srdf_root, args.official_moveit_package)
        check_official_yaml_mapping(audit, project_configs, args.official_moveit_package)

    if audit.failures:
        print('\n[Piper MoveIt2 Config] 审计失败，请先修正上面的不一致。')
        return 2

    print('\n[Piper MoveIt2 Config] 审计通过：官方 Piper URDF/MoveIt2 配置已映射到项目侧 piper_* 链路。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
