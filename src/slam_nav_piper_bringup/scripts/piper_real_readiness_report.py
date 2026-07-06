#!/usr/bin/env python3

import argparse
import json
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_prefix, get_package_share_directory


PROJECT_PACKAGES = [
    'slam_nav_piper_interfaces',
    'slam_nav_piper_description',
    'slam_nav_piper_perception',
    'slam_nav_piper_control',
    'slam_nav_piper_moveit_config',
    'slam_nav_piper_manipulation',
    'slam_nav_piper_calibration',
    'slam_nav_piper_bringup',
]

OFFICIAL_PACKAGES = [
    'piper_description',
    'piper_moveit_config_v4',
    'piper_moveit_config_v5',
]

MOVEIT_PACKAGES = [
    'moveit_ros_planning_interface',
    'moveit_ros_move_group',
    'moveit_planners_ompl',
    'moveit_simple_controller_manager',
]

CONTROL_PACKAGES = [
    'controller_manager',
    'joint_state_broadcaster',
    'joint_trajectory_controller',
    'gripper_controllers',
    'trajectory_msgs',
]


def find_workspace_root():
    current = Path.cwd().resolve()
    for candidate in [current, *current.parents]:
        if (candidate / 'src').exists() and (candidate / '.git').exists():
            return candidate
    return current


def package_file(package_name, relative_path):
    workspace_root = find_workspace_root()
    source_candidate = workspace_root / 'src' / package_name / relative_path
    if source_candidate.exists():
        return source_candidate
    return Path(get_package_share_directory(package_name)) / relative_path


def read_yaml(path):
    with path.open('r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def find_ros_parameters(value):
    if isinstance(value, dict):
        params = value.get('ros__parameters')
        if isinstance(params, dict):
            return params
        for child in value.values():
            result = find_ros_parameters(child)
            if result is not None:
                return result
    return None


def params_from_yaml(path):
    params = find_ros_parameters(read_yaml(path))
    if params is None:
        raise RuntimeError(f'未在 {path} 中找到 ros__parameters。')
    return params


def package_found(package_name):
    try:
        return True, get_package_prefix(package_name)
    except PackageNotFoundError:
        return False, ''


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


class Report:
    def __init__(self):
        self.ok_items = []
        self.wait_items = []
        self.fail_items = []

    def ok(self, message):
        self.ok_items.append(message)

    def wait(self, message):
        self.wait_items.append(message)

    def fail(self, message):
        self.fail_items.append(message)

    def expect(self, condition, ok_message, fail_message):
        if condition:
            self.ok(ok_message)
        else:
            self.fail(fail_message)

    def wait_until(self, condition, ok_message, wait_message):
        if condition:
            self.ok(ok_message)
        else:
            self.wait(wait_message)

    def print_text(self):
        print('[Piper Real Readiness] 实机接入前状态报告')
        print()
        for title, items in (
            ('OK', self.ok_items),
            ('WAIT', self.wait_items),
            ('FAIL', self.fail_items),
        ):
            print(f'[{title}]')
            if items:
                for item in items:
                    print(f'  - {item}')
            else:
                print('  - 无')
            print()

    def as_dict(self):
        return {
            'ok': self.ok_items,
            'wait': self.wait_items,
            'fail': self.fail_items,
        }


def check_packages(report):
    for package_name in PROJECT_PACKAGES:
        found, prefix = package_found(package_name)
        report.expect(found, f'项目侧包 {package_name} 可用: {prefix}', f'缺少项目侧包 {package_name}')

    for package_name in OFFICIAL_PACKAGES:
        found, prefix = package_found(package_name)
        report.expect(found, f'AgileX 官方包 {package_name} 可用: {prefix}', f'缺少 AgileX 官方包 {package_name}')

    for package_name in MOVEIT_PACKAGES:
        found, prefix = package_found(package_name)
        report.expect(found, f'MoveIt2 包 {package_name} 可用: {prefix}', f'缺少 MoveIt2 包 {package_name}')

    for package_name in CONTROL_PACKAGES:
        found, prefix = package_found(package_name)
        report.expect(found, f'ros2_control 包 {package_name} 可用: {prefix}', f'缺少 ros2_control 包 {package_name}')


def check_config(report, workspace_root, args):
    safety = params_from_yaml(package_file('slam_nav_piper_bringup', 'config/piper_safety_limits.yaml'))
    mobile = params_from_yaml(package_file('slam_nav_piper_bringup', 'config/piper_mobile_manipulation.yaml'))
    control = params_from_yaml(package_file('slam_nav_piper_control', 'config/piper_control.yaml'))
    task = params_from_yaml(package_file('slam_nav_piper_manipulation', 'config/piper_manipulation.yaml'))
    hand_eye = params_from_yaml(package_file('slam_nav_piper_calibration', 'config/hand_eye_calibration.yaml'))

    report.expect(not as_bool(safety.get('auto_enable')), '安全默认 auto_enable=false。', '安全默认 auto_enable 不应为 true。')
    report.expect(not as_bool(safety.get('allow_real_motion')), '安全默认 allow_real_motion=false。', '配置阶段 allow_real_motion 不应默认 true。')
    report.expect(
        float(safety.get('velocity_scaling_initial', 1.0)) <= 0.10,
        '实机初始速度比例不超过 0.10。',
        '实机初始速度比例超过 0.10。',
    )
    report.expect(
        bool(safety.get('base_must_be_stopped_during_arm_motion')),
        '机械臂运动期间要求底盘停止。',
        '缺少机械臂运动期间底盘停止要求。',
    )

    forbidden_topics = set(safety.get('forbidden_topics_for_default_nav2_costmap', []))
    required_forbidden = {
        '/piper/arm_camera/depth/points',
        '/piper/perception/target_pose',
        '/piper/grasp_candidates',
        '/piper/learning/grasp_candidates_ranked',
    }
    report.expect(
        required_forbidden.issubset(forbidden_topics),
        'Nav2 默认 costmap 禁止接入 Piper 相机/感知/学习话题。',
        'Nav2 默认 costmap 禁止列表缺少 Piper 话题。',
    )

    report.expect(mobile.get('namespace') == '/piper', '移动操作命名空间为 /piper。', '移动操作命名空间不是 /piper。')
    report.expect(
        mobile.get('arm_camera_namespace') == '/piper/arm_camera',
        '腕部 RGB-D 相机命名空间为 /piper/arm_camera。',
        '腕部 RGB-D 相机命名空间不正确。',
    )
    report.expect(
        mobile.get('navigation_camera_namespace') == '/nav_camera',
        '导航相机命名空间仍保留 /nav_camera，未与 Piper 相机混用。',
        '导航相机命名空间被改动，需确认未混入 Piper 相机。',
    )
    report.expect(
        not as_bool(mobile.get('gazebo_enable_piper_arm_default')),
        'Gazebo 默认不挂载 Piper，task1 仿真默认不变。',
        'Gazebo 默认挂载 Piper，会影响 task1 默认仿真。',
    )
    report.expect(
        bool(mobile.get('moveit_config_ready')),
        '项目侧 MoveIt2 plan-only 配置已标记就绪。',
        '项目侧 MoveIt2 配置未标记就绪。',
    )
    report.wait_until(
        bool(mobile.get('moveit_execution_ready')),
        'MoveIt2 真实执行后端已标记就绪。',
        'MoveIt2 真实执行后端仍未接入，保持 plan-only。',
    )
    report.wait_until(
        bool(mobile.get('sdk_driver_ready')),
        '厂家 SDK/驱动后端已标记就绪。',
        '厂家 SDK/驱动后端仍未接入，仅保留适配边界。',
    )

    report.expect(control.get('backend') == 'moveit', '控制桥默认后端为 moveit。', '控制桥默认后端不是 moveit。')
    report.expect(control.get('initial_owner') == 'disabled', '控制桥初始 owner=disabled。', '控制桥初始 owner 不是 disabled。')
    report.expect(not as_bool(control.get('auto_enable')), '控制桥默认不自动使能。', '控制桥不应默认自动使能。')
    report.expect(not as_bool(control.get('allow_real_motion')), '控制桥默认禁止真实运动。', '控制桥不应默认允许真实运动。')
    report.expect(not as_bool(control.get('allow_sdk_motion_test')), 'SDK 低速运动测试默认关闭。', 'SDK 低速运动测试不应默认打开。')
    report.expect(control.get('moveit_tcp_frame') == 'piper_tcp', 'MoveIt2 TCP frame 为 piper_tcp。', 'MoveIt2 TCP frame 不正确。')
    report.expect(control.get('moveit_base_frame') == 'piper_base_link', 'MoveIt2 base frame 为 piper_base_link。', 'MoveIt2 base frame 不正确。')

    report.expect(bool(task.get('fake_execution')), '任务层默认 fake_execution=true。', '任务层默认不应关闭 fake_execution。')
    report.expect(not as_bool(task.get('real_backend_connected')), '任务层默认 real_backend_connected=false。', '任务层不应默认声明真实后端已接入。')
    report.expect(bool(task.get('require_base_stop_before_motion')), '真实运动前要求底盘停止确认。', '真实运动前未要求底盘停止确认。')
    report.expect(bool(task.get('require_hand_eye_calibration_before_pick')), '真实 pick 前要求手眼标定验收。', '真实 pick 前未要求手眼标定验收。')
    report.expect(not as_bool(task.get('use_ranked_grasp_candidates')), '任务层默认不消费学习排序结果。', '任务层不应默认消费学习排序结果。')

    report.wait_until(
        bool(task.get('real_backend_connected')),
        '任务层已声明真实后端接入。',
        '任务层尚未声明真实后端接入。',
    )
    report.wait_until(
        bool(task.get('hand_eye_calibrated')),
        '任务层已标记手眼标定验收通过。',
        '任务层 hand_eye_calibrated=false，真实 pick 仍会被拒绝。',
    )
    report.wait_until(
        bool(task.get('base_stop_confirmed')) or bool(task.get('publish_base_stop')),
        '底盘停止确认/停车发布链路已满足真实运动条件。',
        '底盘停止确认尚未满足，真实机械臂运动仍会被拒绝。',
    )

    result_path = Path(args.hand_eye_result_path or task.get('hand_eye_result_path') or hand_eye.get('result_yaml_path', ''))
    if not result_path.is_absolute():
        result_path = workspace_root / result_path
    report.wait_until(
        result_path.exists(),
        f'手眼标定结果文件存在: {result_path}',
        f'手眼标定结果文件尚不存在: {result_path}',
    )

    report.expect(not as_bool(hand_eye.get('enabled')), '手眼标定默认 enabled=false。', '手眼标定不应默认 enabled=true。')
    report.expect(not as_bool(hand_eye.get('allow_live_motion')), '手眼标定默认禁止 live motion。', '手眼标定不应默认允许 live motion。')
    report.expect(bool(hand_eye.get('require_manual_result_review')), '手眼标定结果要求人工复核。', '手眼标定结果缺少人工复核要求。')
    report.expect(
        not as_bool(hand_eye.get('publish_calibrated_tf_by_default')),
        '手眼标定默认不发布最终 TF。',
        '手眼标定不应默认发布最终 TF。',
    )
    report.expect(
        not as_bool(hand_eye.get('allow_write_to_robot_description')),
        '手眼标定默认不写 robot_description。',
        '手眼标定不应默认写 robot_description。',
    )


def main():
    parser = argparse.ArgumentParser(description='Piper 实机接入前状态报告。')
    parser.add_argument('--require-ready', action='store_true', help='要求真实执行条件全部满足；否则返回非 0。')
    parser.add_argument('--hand-eye-result-path', default='', help='覆盖默认手眼标定结果路径。')
    parser.add_argument('--json', action='store_true', help='以 JSON 输出报告。')
    args = parser.parse_args()

    workspace_root = find_workspace_root()
    report = Report()

    try:
        check_packages(report)
        check_config(report, workspace_root, args)
    except Exception as exc:
        report.fail(f'读取/检查配置失败: {exc}')

    if args.json:
        print(json.dumps(report.as_dict(), ensure_ascii=False, indent=2))
    else:
        report.print_text()
        if report.wait_items:
            print('[Piper Real Readiness] 当前结论：配置安全，真实执行仍保持未接入/未验收状态。')
        if not report.fail_items and not report.wait_items:
            print('[Piper Real Readiness] 当前结论：真实执行前置条件已全部满足。')

    if report.fail_items:
        return 2
    if args.require_ready and report.wait_items:
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
