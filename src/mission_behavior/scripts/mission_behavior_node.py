#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import BackUp, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node


class BtStatus(Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class MissionBehaviorNode(Node):
    """任务层行为树节点：导航失败时先清理代价地图，再后退脱困并重试。"""

    def __init__(self):
        super().__init__('mission_behavior_node')

        self.declare_parameter('auto_start', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 4.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('max_navigation_retries', 1)
        self.declare_parameter('navigate_timeout_sec', 180.0)
        self.declare_parameter('backup_distance', 0.45)
        self.declare_parameter('backup_speed', 0.12)
        self.declare_parameter('backup_timeout_sec', 8.0)
        self.declare_parameter('replan_wait_sec', 0.5)
        self.declare_parameter('recovery_strategy', 'free_space')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('odom_topic', '/Odometry')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('free_space_distance', 0.45)
        self.declare_parameter('free_space_speed', 0.12)
        self.declare_parameter('free_space_corridor_width', 0.55)
        self.declare_parameter('free_space_sample_step', 0.10)
        self.declare_parameter('occupied_cost_threshold', 65)
        self.declare_parameter('unknown_cost_penalty', 0.25)
        self.declare_parameter('occupied_cost_penalty', 5.0)
        self.declare_parameter('max_occupied_ratio', 0.08)
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('backup_action', '/backup')
        self.declare_parameter(
            'local_clear_service',
            '/local_costmap/clear_entirely_local_costmap',
        )
        self.declare_parameter(
            'global_clear_service',
            '/global_costmap/clear_entirely_global_costmap',
        )

        self.auto_start = bool(self.get_parameter('auto_start').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.goal_x = float(self.get_parameter('goal_x').value)
        self.goal_y = float(self.get_parameter('goal_y').value)
        self.goal_yaw = float(self.get_parameter('goal_yaw').value)
        self.max_navigation_retries = int(self.get_parameter('max_navigation_retries').value)
        self.navigate_timeout_sec = float(self.get_parameter('navigate_timeout_sec').value)
        self.backup_distance = float(self.get_parameter('backup_distance').value)
        self.backup_speed = float(self.get_parameter('backup_speed').value)
        self.backup_timeout_sec = float(self.get_parameter('backup_timeout_sec').value)
        self.replan_wait_sec = float(self.get_parameter('replan_wait_sec').value)
        self.recovery_strategy = str(self.get_parameter('recovery_strategy').value)
        self.free_space_distance = float(self.get_parameter('free_space_distance').value)
        self.free_space_speed = float(self.get_parameter('free_space_speed').value)
        self.free_space_corridor_width = float(
            self.get_parameter('free_space_corridor_width').value
        )
        self.free_space_sample_step = float(self.get_parameter('free_space_sample_step').value)
        self.occupied_cost_threshold = int(self.get_parameter('occupied_cost_threshold').value)
        self.unknown_cost_penalty = float(self.get_parameter('unknown_cost_penalty').value)
        self.occupied_cost_penalty = float(self.get_parameter('occupied_cost_penalty').value)
        self.max_occupied_ratio = float(self.get_parameter('max_occupied_ratio').value)

        self.latest_costmap = None
        self.latest_odom = None

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter('navigate_action').value,
        )
        self.backup_client = ActionClient(
            self,
            BackUp,
            self.get_parameter('backup_action').value,
        )
        self.local_clear_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('local_clear_service').value,
        )
        self.global_clear_client = self.create_client(
            ClearEntireCostmap,
            self.get_parameter('global_clear_service').value,
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter('cmd_vel_topic').value,
            10,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('local_costmap_topic').value,
            self._costmap_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            10,
        )

        self.get_logger().info(
            'Mission behavior tree ready. '
            f'auto_start={self.auto_start}, goal=({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_yaw:.2f})'
        )

    def run_once(self):
        self._log_tree('Sequence', 'MissionNavigation')

        if not self._wait_for_nav2_interfaces():
            self._log_status('MissionNavigation', BtStatus.FAILURE)
            return BtStatus.FAILURE

        for attempt in range(self.max_navigation_retries + 1):
            self.get_logger().info(
                f'[BT] Navigate attempt {attempt + 1}/{self.max_navigation_retries + 1}'
            )
            nav_status = self._navigate_to_configured_goal()
            if nav_status == BtStatus.SUCCESS:
                self._log_status('MissionNavigation', BtStatus.SUCCESS)
                return BtStatus.SUCCESS

            if attempt >= self.max_navigation_retries:
                break

            recovery_status = self._backup_recovery()
            if recovery_status != BtStatus.SUCCESS:
                self.get_logger().warn('[BT] Recovery failed, but mission will still try final fallback.')

        self._log_status('MissionNavigation', BtStatus.FAILURE)
        return BtStatus.FAILURE

    def _wait_for_nav2_interfaces(self):
        self._log_tree('Sequence', 'WaitForNav2Interfaces')
        ready = True
        ready &= self._wait_for_action(self.navigate_client, 'NavigateToPose')
        ready &= self._wait_for_action(self.backup_client, 'BackUp')
        ready &= self._wait_for_service(self.local_clear_client, 'local clear costmap')
        ready &= self._wait_for_service(self.global_clear_client, 'global clear costmap')
        self._log_status('WaitForNav2Interfaces', BtStatus.SUCCESS if ready else BtStatus.FAILURE)
        return ready

    def _navigate_to_configured_goal(self):
        self._log_tree('Action', 'NavigateToPose')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_goal_pose()

        send_future = self.navigate_client.send_goal_async(goal_msg, feedback_callback=self._nav_feedback)
        goal_handle = self._wait_for_future(send_future, 10.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('[BT] NavigateToPose goal was rejected.')
            self._log_status('NavigateToPose', BtStatus.FAILURE)
            return BtStatus.FAILURE

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, self.navigate_timeout_sec)
        if result is None:
            self.get_logger().warn('[BT] NavigateToPose timed out, canceling goal.')
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 3.0)
            self._log_status('NavigateToPose', BtStatus.FAILURE)
            return BtStatus.FAILURE

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self._log_status('NavigateToPose', BtStatus.SUCCESS)
            return BtStatus.SUCCESS

        self.get_logger().warn(f'[BT] NavigateToPose failed with status={result.status}.')
        self._log_status('NavigateToPose', BtStatus.FAILURE)
        return BtStatus.FAILURE

    def _backup_recovery(self):
        self._log_tree('Sequence', 'BackupRecovery')

        if self.recovery_strategy == 'free_space':
            if self._free_space_recovery():
                self._clear_costmaps()
                self._wait_before_replan()
                self._log_status('BackupRecovery', BtStatus.SUCCESS)
                return BtStatus.SUCCESS
            self.get_logger().warn(
                '[BT] Free-space recovery unavailable, falling back to Nav2 BackUp.'
            )

        if not self._clear_costmaps():
            self._log_status('BackupRecovery', BtStatus.FAILURE)
            return BtStatus.FAILURE

        if not self._backup():
            self._log_status('BackupRecovery', BtStatus.FAILURE)
            return BtStatus.FAILURE

        self._wait_before_replan()
        self._log_status('BackupRecovery', BtStatus.SUCCESS)
        return BtStatus.SUCCESS

    def _wait_before_replan(self):
        self._log_tree('Action', 'WaitBeforeReplan')
        time.sleep(self.replan_wait_sec)
        self._log_status('WaitBeforeReplan', BtStatus.SUCCESS)

    def _clear_costmaps(self):
        self._log_tree('Sequence', 'ClearCostmaps')
        local_ok = self._call_clear_service(self.local_clear_client, 'local costmap')
        global_ok = self._call_clear_service(self.global_clear_client, 'global costmap')
        ok = local_ok and global_ok
        self._log_status('ClearCostmaps', BtStatus.SUCCESS if ok else BtStatus.FAILURE)
        return ok

    def _backup(self):
        self._log_tree('Action', 'BackUp')

        goal_msg = BackUp.Goal()
        goal_msg.target = Point(x=-abs(self.backup_distance), y=0.0, z=0.0)
        goal_msg.speed = abs(self.backup_speed)
        goal_msg.time_allowance = Duration(sec=int(self.backup_timeout_sec), nanosec=0)

        send_future = self.backup_client.send_goal_async(goal_msg)
        goal_handle = self._wait_for_future(send_future, 5.0)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('[BT] BackUp goal was rejected.')
            self._log_status('BackUp', BtStatus.FAILURE)
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, self.backup_timeout_sec + 2.0)
        if result is None:
            self.get_logger().warn('[BT] BackUp timed out, canceling goal.')
            cancel_future = goal_handle.cancel_goal_async()
            self._wait_for_future(cancel_future, 3.0)
            self._log_status('BackUp', BtStatus.FAILURE)
            return False

        ok = result.status == GoalStatus.STATUS_SUCCEEDED
        self._log_status('BackUp', BtStatus.SUCCESS if ok else BtStatus.FAILURE)
        return ok

    def _free_space_recovery(self):
        self._log_tree('Action', 'FreeSpaceRecovery')

        direction = self._select_free_space_direction()
        if direction is None:
            self._log_status('FreeSpaceRecovery', BtStatus.FAILURE)
            return False

        label, vx_unit, vy_unit, score = direction
        speed = max(abs(self.free_space_speed), 0.01)
        distance = max(abs(self.free_space_distance), 0.05)
        duration = min(max(distance / speed, 0.5), self.backup_timeout_sec)

        self.get_logger().info(
            '[BT] Free-space recovery selected '
            f'{label}: vx={vx_unit * speed:.2f}, vy={vy_unit * speed:.2f}, '
            f'duration={duration:.2f}s, score={score:.2f}'
        )

        twist = Twist()
        twist.linear.x = vx_unit * speed
        twist.linear.y = vy_unit * speed

        start_time = time.monotonic()
        while rclpy.ok() and time.monotonic() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self._publish_zero_velocity()
        self._log_status('FreeSpaceRecovery', BtStatus.SUCCESS)
        return True

    def _select_free_space_direction(self):
        if self.latest_costmap is None or self.latest_odom is None:
            self.get_logger().warn('[BT] No local costmap or odometry yet for free-space recovery.')
            return None

        candidates = [
            ('backward', -1.0, 0.0, 0.30),
            ('back_left', -0.70, 0.70, 0.20),
            ('back_right', -0.70, -0.70, 0.20),
            ('left', 0.0, 1.0, 0.05),
            ('right', 0.0, -1.0, 0.05),
        ]

        best = None
        for label, vx, vy, prior in candidates:
            norm = math.hypot(vx, vy)
            if norm <= 1e-6:
                continue
            vx_unit = vx / norm
            vy_unit = vy / norm
            score, occupied_ratio = self._score_recovery_direction(vx_unit, vy_unit)
            if occupied_ratio > self.max_occupied_ratio:
                continue
            score += prior
            if best is None or score > best[3]:
                best = (label, vx_unit, vy_unit, score)

        return best

    def _score_recovery_direction(self, vx_unit, vy_unit):
        costmap = self.latest_costmap
        odom = self.latest_odom
        info = costmap.info

        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dir_x = cos_yaw * vx_unit - sin_yaw * vy_unit
        dir_y = sin_yaw * vx_unit + cos_yaw * vy_unit
        lateral_x = -dir_y
        lateral_y = dir_x

        sample_step = max(self.free_space_sample_step, info.resolution)
        half_width = max(self.free_space_corridor_width * 0.5, info.resolution)
        lateral_offsets = [-half_width, 0.0, half_width]

        free_count = 0
        unknown_count = 0
        occupied_count = 0
        sample_count = 0

        distance = sample_step
        while distance <= max(abs(self.free_space_distance), sample_step):
            for offset in lateral_offsets:
                x = robot_x + dir_x * distance + lateral_x * offset
                y = robot_y + dir_y * distance + lateral_y * offset
                cost = self._cost_at_world(x, y)
                sample_count += 1
                if cost is None or cost < 0:
                    unknown_count += 1
                elif cost >= self.occupied_cost_threshold:
                    occupied_count += 1
                else:
                    free_count += 1
            distance += sample_step

        if sample_count == 0:
            return -math.inf, 1.0

        occupied_ratio = occupied_count / sample_count
        score = (
            free_count
            - unknown_count * self.unknown_cost_penalty
            - occupied_count * self.occupied_cost_penalty
        ) / sample_count
        return score, occupied_ratio

    def _cost_at_world(self, x, y):
        costmap = self.latest_costmap
        info = costmap.info
        origin = info.origin.position
        col = int((x - origin.x) / info.resolution)
        row = int((y - origin.y) / info.resolution)
        if col < 0 or row < 0 or col >= info.width or row >= info.height:
            return None
        index = row * info.width + col
        if index < 0 or index >= len(costmap.data):
            return None
        return costmap.data[index]

    def _publish_zero_velocity(self):
        zero = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(zero)
            time.sleep(0.02)

    def _costmap_callback(self, msg):
        self.latest_costmap = msg

    def _odom_callback(self, msg):
        self.latest_odom = msg

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _make_goal_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.goal_x
        pose.pose.position.y = self.goal_y
        pose.pose.position.z = 0.0

        half_yaw = self.goal_yaw * 0.5
        pose.pose.orientation.z = math.sin(half_yaw)
        pose.pose.orientation.w = math.cos(half_yaw)
        return pose

    def _call_clear_service(self, client, label):
        self._log_tree('Action', f'Clear {label}')
        request = ClearEntireCostmap.Request()
        future = client.call_async(request)
        result = self._wait_for_future(future, 5.0)
        ok = result is not None
        self._log_status(f'Clear {label}', BtStatus.SUCCESS if ok else BtStatus.FAILURE)
        return ok

    def _wait_for_action(self, client, label):
        self.get_logger().info(f'[BT] Waiting for {label} action server...')
        ok = client.wait_for_server(timeout_sec=10.0)
        if not ok:
            self.get_logger().error(f'[BT] {label} action server is not available.')
        return ok

    def _wait_for_service(self, client, label):
        self.get_logger().info(f'[BT] Waiting for {label} service...')
        ok = client.wait_for_service(timeout_sec=10.0)
        if not ok:
            self.get_logger().error(f'[BT] {label} service is not available.')
        return ok

    def _wait_for_future(self, future, timeout_sec):
        start_time = time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result()
            if time.monotonic() - start_time > timeout_sec:
                return None
        return None

    def _nav_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            '[BT] Navigate feedback: '
            f'distance_remaining={feedback.distance_remaining:.2f}, '
            f'recoveries={feedback.number_of_recoveries}',
            throttle_duration_sec=3.0,
        )

    def _log_tree(self, node_type, name):
        self.get_logger().info(f'[BT] {node_type}: {name}')

    def _log_status(self, name, status):
        self.get_logger().info(f'[BT] {name} -> {status.value}')


def main():
    rclpy.init()
    node = MissionBehaviorNode()
    try:
        if node.auto_start:
            node.run_once()
        else:
            node.get_logger().info(
                'Set auto_start:=true with goal_x/goal_y/goal_yaw to execute the demo mission.'
            )
            rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
