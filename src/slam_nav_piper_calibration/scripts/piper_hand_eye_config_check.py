#!/usr/bin/env python3

import argparse
import sys
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


EXPECTED_CAMERA_TOPICS = {
    'color_image_topic': '/piper/arm_camera/color/image_raw',
    'color_camera_info_topic': '/piper/arm_camera/color/camera_info',
    'depth_image_topic': '/piper/arm_camera/depth/image_raw',
    'depth_camera_info_topic': '/piper/arm_camera/depth/camera_info',
}

EXPECTED_FRAMES = {
    'robot_base_frame': 'piper_base_link',
    'tcp_frame': 'piper_tcp',
    'camera_link_frame': 'piper_arm_camera_link',
    'camera_frame': 'piper_arm_camera_optical_frame',
}

REQUIRED_TRUE = [
    'require_base_stopped',
    'require_estop_available',
    'require_control_owner_disabled_after_collection',
    'require_moveit_plan_only_before_collection',
    'require_manual_result_review',
]

REQUIRED_FALSE = [
    'enabled',
    'allow_live_motion',
    'publish_calibrated_tf_by_default',
    'allow_write_to_robot_description',
]


def find_workspace_root():
    current = Path.cwd().resolve()
    for candidate in [current, *current.parents]:
        if (candidate / 'src').exists() and (candidate / '.git').exists():
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


def default_config_path():
    source_path = source_package_path('slam_nav_piper_calibration')
    if source_path is not None:
        return source_path / 'config' / 'hand_eye_calibration.yaml'
    return package_share_path('slam_nav_piper_calibration') / 'config' / 'hand_eye_calibration.yaml'


def default_perception_config_path():
    source_path = source_package_path('slam_nav_piper_perception')
    if source_path is not None:
        return source_path / 'config' / 'perception.yaml'
    return package_share_path('slam_nav_piper_perception') / 'config' / 'perception.yaml'


def load_params(path):
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    params = data.get('piper_hand_eye_calibration', {}).get('ros__parameters', {})
    if not isinstance(params, dict):
        raise RuntimeError(f'{path} 缺少 piper_hand_eye_calibration.ros__parameters。')
    return params


def iter_strings(value):
    if isinstance(value, dict):
        for item in value.values():
            yield from iter_strings(item)
    elif isinstance(value, list):
        for item in value:
            yield from iter_strings(item)
    elif isinstance(value, str):
        yield value


def load_perception_params(path):
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    target = (
        data.get('piper', {})
        .get('target_pose_estimator_node', {})
        .get('ros__parameters', {})
    )
    fake_camera = (
        data.get('piper', {})
        .get('arm_camera_fake_node', {})
        .get('ros__parameters', {})
    )
    return target if isinstance(target, dict) else {}, fake_camera if isinstance(fake_camera, dict) else {}


class Audit:
    def __init__(self):
        self.failures = []

    def ok(self, message):
        print(f'[Piper HandEye] OK   {message}')

    def fail(self, message):
        self.failures.append(message)
        print(f'[Piper HandEye] FAIL {message}')

    def expect(self, condition, ok_message, fail_message):
        if condition:
            self.ok(ok_message)
        else:
            self.fail(fail_message)


def check_topic_boundaries(audit, params):
    audit.expect(
        params.get('namespace') == '/piper',
        '标定命名空间固定为 /piper。',
        f'namespace 应为 /piper，实际为 {params.get("namespace")}',
    )
    audit.expect(
        params.get('camera_namespace') == '/piper/arm_camera',
        '标定相机命名空间固定为 /piper/arm_camera。',
        f'camera_namespace 应为 /piper/arm_camera，实际为 {params.get("camera_namespace")}',
    )

    for key, expected in EXPECTED_CAMERA_TOPICS.items():
        audit.expect(
            params.get(key) == expected,
            f'{key} 使用 Piper 腕部相机话题。',
            f'{key} 应为 {expected}，实际为 {params.get(key)}',
        )

    calibration_topics = [
        'marker_observation_topic',
        'sample_pose_topic',
        'result_transform_topic',
        'capture_sample_service',
        'solve_service',
    ]
    bad_topics = [
        f'{key}={params.get(key)}'
        for key in calibration_topics
        if not str(params.get(key, '')).startswith('/piper/calibration/')
    ]
    audit.expect(
        not bad_topics,
        '标定输出和服务都隔离在 /piper/calibration 下。',
        '标定 topic/service 不在 /piper/calibration 下: ' + ', '.join(bad_topics),
    )

    nav_camera_refs = [item for item in iter_strings(params) if '/nav_camera' in item]
    audit.expect(
        not nav_camera_refs,
        '手眼标定配置没有引用 /nav_camera。',
        '手眼标定配置不应引用 /nav_camera: ' + ', '.join(nav_camera_refs),
    )


def check_frames(audit, params):
    audit.expect(
        params.get('calibration_type') == 'eye_in_hand' and params.get('camera_mount') == 'wrist',
        '标定类型固定为腕部 eye-in-hand。',
        (
            '标定类型应为 eye_in_hand/wrist，'
            f'实际为 {params.get("calibration_type")}/{params.get("camera_mount")}'
        ),
    )
    for key, expected in EXPECTED_FRAMES.items():
        audit.expect(
            params.get(key) == expected,
            f'{key}={expected}',
            f'{key} 应为 {expected}，实际为 {params.get(key)}',
        )

    target_frame = str(params.get('calibration_target_frame', ''))
    audit.expect(
        target_frame.startswith('piper_') and target_frame != params.get('camera_frame'),
        '标定板 frame 使用独立 piper_* frame。',
        f'calibration_target_frame 不合理: {target_frame}',
    )


def check_board_and_sampling(audit, params):
    board_type = params.get('board_type')
    audit.expect(
        board_type in {'charuco', 'apriltag_grid', 'checkerboard'},
        '标定板类型是受支持的棋盘/AprilTag 类别。',
        f'不支持的 board_type: {board_type}',
    )

    numeric_positive = [
        'board_squares_x',
        'board_squares_y',
        'board_square_size_m',
        'board_marker_size_m',
        'min_translation_span_m',
        'min_rotation_span_deg',
        'max_reprojection_error_px',
    ]
    invalid = [
        key
        for key in numeric_positive
        if not isinstance(params.get(key), (int, float)) or float(params.get(key)) <= 0.0
    ]
    audit.expect(
        not invalid,
        '标定板尺寸、覆盖范围和重投影阈值均为正数。',
        '标定数值参数必须为正数: ' + ', '.join(invalid),
    )

    min_samples = params.get('min_sample_count')
    audit.expect(
        isinstance(min_samples, int) and min_samples >= 12,
        '手眼标定最少采样数不少于 12 组。',
        f'min_sample_count 应为 >=12 的整数，实际为 {min_samples}',
    )


def check_safety_flags(audit, params):
    for key in REQUIRED_TRUE:
        audit.expect(
            params.get(key) is True,
            f'安全开关 {key}=true。',
            f'安全开关 {key} 应为 true，实际为 {params.get(key)}',
        )
    for key in REQUIRED_FALSE:
        audit.expect(
            params.get(key) is False,
            f'安全开关 {key}=false。',
            f'安全开关 {key} 应为 false，实际为 {params.get(key)}',
        )


def check_output_paths(audit, params):
    output_keys = ['output_directory', 'result_yaml_path', 'sample_bag_directory']
    values = {key: str(params.get(key, '')) for key in output_keys}
    bad_paths = [
        f'{key}={value}'
        for key, value in values.items()
        if value.startswith('/') or value.startswith('src/') or '..' in Path(value).parts
    ]
    audit.expect(
        not bad_paths,
        '标定输出使用工作区内相对路径，且不写入 src/。',
        '标定输出路径不安全: ' + ', '.join(bad_paths),
    )

    non_dataset_paths = [
        f'{key}={value}'
        for key, value in values.items()
        if not value.startswith('datasets/')
    ]
    audit.expect(
        not non_dataset_paths,
        '标定样本和结果默认写入 datasets/，避免进 Git。',
        '标定输出默认应放入 datasets/: ' + ', '.join(non_dataset_paths),
    )

    workspace_root = find_workspace_root()
    if workspace_root is None:
        audit.ok('未找到工作区根目录，跳过 .gitignore 检查。')
        return
    gitignore = workspace_root / '.gitignore'
    if not gitignore.exists():
        audit.fail('未找到 .gitignore，无法确认 datasets/ 是否忽略。')
        return
    text = gitignore.read_text(encoding='utf-8')
    audit.expect(
        'datasets/' in text,
        '.gitignore 已忽略 datasets/ 标定数据目录。',
        '.gitignore 缺少 datasets/，标定数据可能进入 Git。',
    )


def check_perception_alignment(audit, params, perception_path):
    if not perception_path.exists():
        audit.fail(f'未找到 Piper perception 配置: {perception_path}')
        return

    target_params, fake_camera_params = load_perception_params(perception_path)
    comparisons = {
        'color_image_topic': fake_camera_params.get('color_image_topic'),
        'color_camera_info_topic': fake_camera_params.get('color_camera_info_topic'),
        'depth_image_topic': target_params.get('depth_image_topic'),
        'depth_camera_info_topic': target_params.get('depth_camera_info_topic'),
    }
    mismatches = [
        f'{key}: calibration={params.get(key)}, perception={value}'
        for key, value in comparisons.items()
        if params.get(key) != value
    ]
    audit.expect(
        not mismatches,
        '手眼标定相机输入与 Piper perception 配置保持一致。',
        '手眼标定相机输入与 perception 配置不一致: ' + ', '.join(mismatches),
    )


def main():
    parser = argparse.ArgumentParser(description='检查 Piper 腕部 RGB-D 手眼标定配置边界。')
    parser.add_argument('--config', default='', help='可选：手眼标定 YAML 路径。')
    parser.add_argument('--perception-config', default='', help='可选：Piper perception YAML 路径。')
    args = parser.parse_args()

    try:
        config_path = Path(args.config).resolve() if args.config else default_config_path()
        perception_path = (
            Path(args.perception_config).resolve()
            if args.perception_config else
            default_perception_config_path()
        )
    except PackageNotFoundError as exc:
        print(f'[Piper HandEye] FAIL 缺少已安装包: {exc}', file=sys.stderr)
        return 2

    audit = Audit()
    print(f'[Piper HandEye] 标定配置: {config_path}')
    print(f'[Piper HandEye] 感知配置: {perception_path}')

    try:
        params = load_params(config_path)
    except Exception as exc:
        print(f'[Piper HandEye] FAIL 读取配置失败: {exc}', file=sys.stderr)
        return 2

    check_topic_boundaries(audit, params)
    check_frames(audit, params)
    check_board_and_sampling(audit, params)
    check_safety_flags(audit, params)
    check_output_paths(audit, params)
    check_perception_alignment(audit, params, perception_path)

    if audit.failures:
        print('\n[Piper HandEye] 手眼标定配置检查失败。')
        return 2

    print('\n[Piper HandEye] 手眼标定配置检查通过。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
