#!/usr/bin/env python3

import sys
from typing import List

import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node


class PiperPlanSmokeTest(Node):
    """向 MoveIt2 发送一次 Piper plan-only 规划请求，不执行轨迹。"""

    def __init__(self):
        super().__init__('piper_plan_smoke_test')
        self.declare_parameter('plan_service', '/piper/plan_kinematic_path')
        self.declare_parameter('group_name', 'piper_arm')
        self.declare_parameter('planner_id', 'RRTConnect')
        self.declare_parameter('joint_names', [
            'piper_joint1',
            'piper_joint2',
            'piper_joint3',
            'piper_joint4',
            'piper_joint5',
            'piper_joint6',
        ])
        self.declare_parameter('target_positions', [0.0, 0.4164, -0.5409, 0.0, 0.0, 0.0])
        self.declare_parameter('joint_tolerance', 0.02)
        self.declare_parameter('allowed_planning_time', 5.0)
        self.declare_parameter('num_planning_attempts', 5)
        self.declare_parameter('service_timeout_s', 20.0)

        self.plan_service = str(self.get_parameter('plan_service').value)
        self.client = self.create_client(GetMotionPlan, self.plan_service)

    def run(self):
        timeout_s = float(self.get_parameter('service_timeout_s').value)
        self.get_logger().info(f'等待 MoveIt2 规划服务: {self.plan_service}')
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error(f'等待规划服务超时: {self.plan_service}')
            return 2

        request = self.build_request()
        self.get_logger().info(
            f'发送 plan-only 请求: group={request.motion_plan_request.group_name}, '
            f'planner={request.motion_plan_request.planner_id}'
        )
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done():
            self.get_logger().error('规划请求超时。')
            return 2

        response = future.result()
        if response is None:
            self.get_logger().error('规划服务没有返回结果。')
            return 2

        motion_response = response.motion_plan_response
        error_code = motion_response.error_code.val
        point_count = len(motion_response.trajectory.joint_trajectory.points)
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'规划失败，MoveIt error_code={error_code}。')
            return 2
        if point_count == 0:
            self.get_logger().error('规划返回成功但轨迹点为空。')
            return 2

        self.get_logger().info(
            f'规划成功：轨迹点数={point_count}, planning_time={motion_response.planning_time:.3f}s'
        )
        return 0

    def build_request(self):
        joint_names = [str(item) for item in self.get_parameter('joint_names').value]
        target_positions = [float(item) for item in self.get_parameter('target_positions').value]
        if len(joint_names) != len(target_positions):
            raise RuntimeError('joint_names 与 target_positions 长度必须一致。')

        request = GetMotionPlan.Request()
        motion_request = request.motion_plan_request
        motion_request.group_name = str(self.get_parameter('group_name').value)
        motion_request.pipeline_id = 'ompl'
        motion_request.planner_id = str(self.get_parameter('planner_id').value)
        motion_request.num_planning_attempts = int(self.get_parameter('num_planning_attempts').value)
        motion_request.allowed_planning_time = float(self.get_parameter('allowed_planning_time').value)
        motion_request.max_velocity_scaling_factor = 0.1
        motion_request.max_acceleration_scaling_factor = 0.1
        # 让 MoveIt 使用当前状态监听器里的起点；本测试只验证规划，不执行轨迹。
        motion_request.start_state.is_diff = True
        motion_request.goal_constraints = [self.build_goal_constraints(joint_names, target_positions)]
        return request

    def build_goal_constraints(self, joint_names: List[str], target_positions: List[float]):
        tolerance = float(self.get_parameter('joint_tolerance').value)
        constraints = Constraints()
        constraints.name = 'piper_ready_joint_goal'
        for joint_name, target_position in zip(joint_names, target_positions):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = target_position
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints


def main():
    rclpy.init()
    node = PiperPlanSmokeTest()
    try:
        exit_code = node.run()
    except Exception as exc:
        node.get_logger().error(f'规划冒烟测试异常: {exc}')
        exit_code = 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
