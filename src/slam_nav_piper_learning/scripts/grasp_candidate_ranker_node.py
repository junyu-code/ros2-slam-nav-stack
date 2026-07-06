#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from slam_nav_piper_interfaces.msg import GraspCandidateArray


class GraspCandidateRankerNode(Node):
    """Piper 学习策略占位节点：只做候选排序，不直接控制机械臂。"""

    VALID_BACKENDS = {'disabled', 'heuristic', 'rl', 'onnx'}

    def __init__(self):
        super().__init__('grasp_candidate_ranker_node')
        self.declare_parameter('policy_backend', 'disabled')
        self.declare_parameter('input_candidates_topic', '/piper/grasp_candidates')
        self.declare_parameter('output_candidates_topic', '/piper/learning/grasp_candidates_ranked')
        self.declare_parameter('policy_model_path', '')
        self.declare_parameter('min_score', 0.0)
        self.declare_parameter('max_candidates', 8)
        self.declare_parameter('publish_passthrough_when_disabled', False)
        self.declare_parameter('stamp_ranked_candidates', True)
        self.declare_parameter('dataset_root', 'datasets/piper')
        self.declare_parameter('checkpoint_root', 'checkpoints/piper')
        self.declare_parameter('tensorboard_root', 'runs/piper')

        self.policy_backend = self.normalize_backend(str(self.get_parameter('policy_backend').value))
        self.min_score = float(self.get_parameter('min_score').value)
        self.max_candidates = max(int(self.get_parameter('max_candidates').value), 1)
        self.publish_passthrough_when_disabled = bool(
            self.get_parameter('publish_passthrough_when_disabled').value
        )
        self.stamp_ranked_candidates = bool(self.get_parameter('stamp_ranked_candidates').value)

        self.publisher = self.create_publisher(
            GraspCandidateArray,
            str(self.get_parameter('output_candidates_topic').value),
            10,
        )
        self.create_subscription(
            GraspCandidateArray,
            str(self.get_parameter('input_candidates_topic').value),
            self.candidates_callback,
            10,
        )

        self.get_logger().info(
            f'Piper 学习候选排序节点已启动，backend={self.policy_backend}。'
        )
        if self.policy_backend in {'rl', 'onnx'}:
            self.get_logger().warn('RL/ONNX 后端当前只是接口占位，不会加载模型或控制机械臂。')

    def normalize_backend(self, backend):
        if backend not in self.VALID_BACKENDS:
            self.get_logger().warn(f'未知学习后端 {backend}，回退为 disabled。')
            return 'disabled'
        return backend

    def candidates_callback(self, msg):
        if self.policy_backend == 'disabled' and not self.publish_passthrough_when_disabled:
            return

        ranked = copy.deepcopy(msg)
        filtered = [
            candidate
            for candidate in ranked.candidates
            if float(candidate.score) >= self.min_score
        ]

        # 当前只提供确定性启发式排序：按候选分数从高到低，真实 RL 后端后续替换这里。
        filtered.sort(key=lambda candidate: float(candidate.score), reverse=True)
        ranked.candidates = filtered[:self.max_candidates]

        if self.stamp_ranked_candidates:
            for index, candidate in enumerate(ranked.candidates):
                candidate.tags.append(f'rank_{index}')
                candidate.tags.append(f'learning_backend_{self.policy_backend}')

        self.publisher.publish(ranked)


def main():
    rclpy.init()
    node = GraspCandidateRankerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
