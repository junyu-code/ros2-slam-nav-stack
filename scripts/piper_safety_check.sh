#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${WORKSPACE_DIR}"

source "${SCRIPT_DIR}/setup_workspace_env.sh"
set -u

python3 - <<'PY'
import sys
from pathlib import Path

import yaml


ROOT = Path.cwd()
failures = []


def fail(message):
    failures.append(message)
    print(f'[Piper Safety] FAIL {message}')


def ok(message):
    print(f'[Piper Safety] OK   {message}')


def load_params(relative_path, top_key, node_key=None):
    path = ROOT / relative_path
    if not path.exists():
        fail(f'配置文件不存在: {path}')
        return {}
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    value = data.get(top_key, {})
    if node_key:
        value = value.get(node_key, {})
    params = value.get('ros__parameters', {})
    if not isinstance(params, dict):
        fail(f'{relative_path} 缺少 ros__parameters。')
        return {}
    return params


def expect_equal(params, key, expected, label):
    actual = params.get(key)
    if actual == expected:
        ok(f'{label}: {key}={expected}')
    else:
        fail(f'{label}: {key} 应为 {expected}，实际为 {actual}')


def expect_true(params, key, label):
    expect_equal(params, key, True, label)


def expect_false(params, key, label):
    expect_equal(params, key, False, label)


def expect_number_le(params, key, maximum, label):
    actual = params.get(key)
    if isinstance(actual, (int, float)) and 0.0 < float(actual) <= maximum:
        ok(f'{label}: {key}={actual} <= {maximum}')
    else:
        fail(f'{label}: {key} 应在 (0, {maximum}]，实际为 {actual}')


def expect_xyz_bounds(params, min_key, max_key, label):
    min_xyz = params.get(min_key)
    max_xyz = params.get(max_key)
    if (
        isinstance(min_xyz, list)
        and isinstance(max_xyz, list)
        and len(min_xyz) == 3
        and len(max_xyz) == 3
        and all(isinstance(value, (int, float)) for value in [*min_xyz, *max_xyz])
        and all(float(min_value) < float(max_value) for min_value, max_value in zip(min_xyz, max_xyz))
    ):
        ok(f'{label}: {min_key} < {max_key}')
    else:
        fail(f'{label}: workspace 边界无效，{min_key}={min_xyz}, {max_key}={max_xyz}')


def expect_forbidden_topics(params):
    topics = params.get('forbidden_topics_for_default_nav2_costmap', [])
    required = {
        '/piper/arm_camera/depth/points',
        '/piper/perception/detections_3d',
        '/piper/perception/target_pose',
        '/piper/grasp_candidates',
        '/piper/visualization/grasp_candidates',
        '/piper/learning/grasp_candidates_ranked',
    }
    if isinstance(topics, list) and required.issubset(set(topics)):
        ok('安全限制: 默认 Nav2 costmap 禁止接入 Piper 感知/学习话题。')
    else:
        fail(f'安全限制: forbidden_topics_for_default_nav2_costmap 缺少必要项，实际为 {topics}')


safety = load_params(
    'src/slam_nav_piper_bringup/config/piper_safety_limits.yaml',
    'piper_safety_limits',
)
mobile = load_params(
    'src/slam_nav_piper_bringup/config/piper_mobile_manipulation.yaml',
    'piper_mobile_manipulation',
)
control = load_params(
    'src/slam_nav_piper_control/config/piper_control.yaml',
    'piper',
    'piper_control_bridge_node',
)
manipulation = load_params(
    'src/slam_nav_piper_manipulation/config/piper_manipulation.yaml',
    'piper',
    'piper_task_server_node',
)
learning = load_params(
    'src/slam_nav_piper_learning/config/piper_learning.yaml',
    'piper',
    'grasp_candidate_ranker_node',
)
hand_eye = load_params(
    'src/slam_nav_piper_calibration/config/hand_eye_calibration.yaml',
    'piper_hand_eye_calibration',
)

expect_false(safety, 'auto_enable', '安全限制')
expect_false(safety, 'allow_real_motion', '安全限制')
expect_true(safety, 'require_estop_test_before_motion', '安全限制')
expect_true(safety, 'require_disable_test_before_motion', '安全限制')
expect_true(safety, 'require_home_test_before_motion', '安全限制')
expect_true(safety, 'require_hand_eye_calibration_before_pick', '安全限制')
expect_true(safety, 'base_must_be_stopped_during_arm_motion', '安全限制')
expect_number_le(safety, 'velocity_scaling_initial', 0.10, '安全限制')
expect_number_le(safety, 'acceleration_scaling_initial', 0.10, '安全限制')
expect_xyz_bounds(safety, 'workspace_min_xyz', 'workspace_max_xyz', '安全限制')
expect_forbidden_topics(safety)

expect_equal(mobile, 'namespace', '/piper', '移动操作边界')
expect_equal(mobile, 'arm_camera_namespace', '/piper/arm_camera', '移动操作边界')
expect_equal(mobile, 'navigation_camera_namespace', '/nav_camera', '移动操作边界')
expect_equal(mobile, 'default_control_backend', 'moveit', '移动操作边界')
expect_false(mobile, 'real_robot_auto_enable', '移动操作边界')
expect_false(mobile, 'publish_base_stop_by_default', '移动操作边界')
expect_false(mobile, 'gazebo_enable_piper_arm_default', '移动操作边界')
expect_equal(mobile, 'gazebo_enable_piper_camera_arg', 'enable_piper_gazebo_camera', '移动操作边界')
expect_false(mobile, 'gazebo_enable_piper_camera_default', '移动操作边界')
expect_false(mobile, 'launch_start_description_default', '移动操作边界')
expect_true(mobile, 'standalone_publish_joint_states_default', '移动操作边界')
expect_true(mobile, 'moveit_config_ready', '移动操作边界')
expect_false(mobile, 'moveit_execution_ready', '移动操作边界')
expect_false(mobile, 'sdk_driver_ready', '移动操作边界')
expect_false(mobile, 'learning_ready', '移动操作边界')
expect_equal(mobile, 'sdk_driver_namespace', '/piper', '移动操作边界')
expect_equal(mobile, 'learning_ranked_candidates_topic', '/piper/learning/grasp_candidates_ranked', '移动操作边界')
expect_false(mobile, 'hand_eye_calibration_ready', '移动操作边界')
expect_equal(mobile, 'hand_eye_calibration_package', 'slam_nav_piper_calibration', '移动操作边界')

expect_equal(control, 'backend', 'moveit', '控制桥')
expect_equal(control, 'initial_owner', 'disabled', '控制桥')
expect_false(control, 'auto_enable', '控制桥')
expect_false(control, 'allow_sdk_motion_test', '控制桥')
expect_false(control, 'allow_real_motion', '控制桥')
expect_number_le(control, 'velocity_scaling', 0.10, '控制桥')
expect_number_le(control, 'acceleration_scaling', 0.10, '控制桥')
expect_xyz_bounds(control, 'workspace_min_xyz', 'workspace_max_xyz', '控制桥')
expect_equal(control, 'moveit_config_package', 'slam_nav_piper_moveit_config', '控制桥')
expect_equal(control, 'moveit_move_group_namespace', '/piper', '控制桥')
expect_equal(control, 'moveit_planning_group', 'piper_arm', '控制桥')
expect_equal(control, 'moveit_tcp_frame', 'piper_tcp', '控制桥')
expect_equal(control, 'moveit_base_frame', 'piper_base_link', '控制桥')
expect_equal(control, 'sdk_driver_namespace', '/piper', '控制桥')

expect_false(manipulation, 'publish_base_stop', '任务层')
expect_true(manipulation, 'require_base_stop_before_motion', '任务层')
expect_false(manipulation, 'base_stop_confirmed', '任务层')
expect_true(manipulation, 'fake_execution', '任务层')
expect_false(manipulation, 'real_backend_connected', '任务层')
expect_true(manipulation, 'require_hand_eye_calibration_before_pick', '任务层')
expect_false(manipulation, 'hand_eye_calibrated', '任务层')
expect_true(manipulation, 'hand_eye_result_must_exist', '任务层')
expect_false(manipulation, 'require_moveit_plan_before_fake_execution', '任务层')
expect_equal(manipulation, 'moveit_plan_service', '/piper/plan_kinematic_path', '任务层')
expect_equal(manipulation, 'moveit_planning_group', 'piper_arm', '任务层')
expect_false(manipulation, 'use_ranked_grasp_candidates', '任务层')
expect_equal(manipulation, 'target_pose_topic', '/piper/perception/target_pose', '任务层')
expect_equal(manipulation, 'grasp_candidates_topic', '/piper/grasp_candidates', '任务层')
expect_equal(manipulation, 'visualization_markers_topic', '/piper/visualization/grasp_candidates', '任务层')
expect_equal(manipulation, 'control_state_topic', '/piper/control/state', '任务层')
expect_equal(manipulation, 'owner_request_topic', '/piper/control/owner_request', '任务层')

expect_equal(learning, 'policy_backend', 'disabled', '学习层')
expect_false(learning, 'publish_passthrough_when_disabled', '学习层')
expect_equal(learning, 'input_candidates_topic', '/piper/grasp_candidates', '学习层')
expect_equal(learning, 'output_candidates_topic', '/piper/learning/grasp_candidates_ranked', '学习层')

expect_false(hand_eye, 'enabled', '手眼标定')
expect_equal(hand_eye, 'calibration_type', 'eye_in_hand', '手眼标定')
expect_equal(hand_eye, 'camera_namespace', '/piper/arm_camera', '手眼标定')
expect_equal(hand_eye, 'robot_base_frame', 'piper_base_link', '手眼标定')
expect_equal(hand_eye, 'tcp_frame', 'piper_tcp', '手眼标定')
expect_equal(hand_eye, 'camera_frame', 'piper_arm_camera_optical_frame', '手眼标定')
expect_false(hand_eye, 'allow_live_motion', '手眼标定')
expect_true(hand_eye, 'require_base_stopped', '手眼标定')
expect_true(hand_eye, 'require_estop_available', '手眼标定')
expect_true(hand_eye, 'require_manual_result_review', '手眼标定')
expect_false(hand_eye, 'publish_calibrated_tf_by_default', '手眼标定')
expect_false(hand_eye, 'allow_write_to_robot_description', '手眼标定')

if failures:
    print()
    print('[Piper Safety] 安全配置检查失败。')
    sys.exit(2)

print()
print('[Piper Safety] 安全配置检查通过。')
PY
