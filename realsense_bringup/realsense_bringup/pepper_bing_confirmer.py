from collections import deque
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


class PepperBingConfirmer(Node):
    def __init__(self):
        super().__init__('pepper_bing_confirmer')

        self.declare_parameter('input_topic', '/pepper/target_point')
        self.declare_parameter('output_topic', '/pepper/confirmed_point_camera')
        self.declare_parameter('confirm_frames', 5)
        self.declare_parameter('stability_radius_m', 0.015)
        self.declare_parameter('execution_busy_topic', '/grab/execution_busy')
        self.declare_parameter('wait_completion_timeout_sec', 120.0)

        self.confirmed_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter('input_topic').value,
            self.point_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('execution_busy_topic').value,
            self.execution_busy_callback,
            10,
        )
        self.create_timer(0.5, self.check_wait_timeout)

        confirm_frames = self.get_confirm_frames()
        self.recent_points = deque(maxlen=confirm_frames)

        self.waiting_for_execution_complete = False
        self.wait_start_time_ns = 0
        self.last_execution_busy = False
        self.seen_execution_busy_true = False

        self.get_logger().info(
            'Pepper bing confirmer ready. '
            f'input={self.get_parameter("input_topic").value}, '
            f'output={self.get_parameter("output_topic").value}, '
            f'confirm_frames={confirm_frames}, '
            f'stability_radius_m={float(self.get_parameter("stability_radius_m").value):.4f}, '
            f'execution_busy_topic={self.get_parameter("execution_busy_topic").value}, '
            f'wait_completion_timeout_sec='
            f'{float(self.get_parameter("wait_completion_timeout_sec").value):.1f}'
        )

    def get_confirm_frames(self):
        return max(1, int(self.get_parameter('confirm_frames').value))

    def point_callback(self, msg):
        point = (float(msg.point.x), float(msg.point.y), float(msg.point.z))

        if not msg.header.frame_id:
            self.get_logger().warn('Received /pepper/target_point with empty frame_id; ignore.')
            return

        self.refresh_window_size()

        if self.waiting_for_execution_complete:
            return

        self.update_confirm_state(msg, point)

    def refresh_window_size(self):
        target_len = self.get_confirm_frames()
        if self.recent_points.maxlen == target_len:
            return
        self.recent_points = deque(self.recent_points, maxlen=target_len)

    def update_confirm_state(self, msg, point):
        if self.recent_points:
            latest_frame = self.recent_points[-1]['header'].frame_id
            if latest_frame != msg.header.frame_id:
                self.get_logger().warn(
                    f'Input frame changed: {latest_frame} -> {msg.header.frame_id}. Reset stability window.'
                )
                self.recent_points.clear()

        self.recent_points.append({'point': point, 'header': msg.header})

        if len(self.recent_points) < self.get_confirm_frames():
            return

        if not self.is_window_stable():
            return

        confirmed_entry = self.recent_points[-1]
        confirmed_msg = PointStamped()
        confirmed_msg.header = confirmed_entry['header']
        confirmed_msg.point.x = confirmed_entry['point'][0]
        confirmed_msg.point.y = confirmed_entry['point'][1]
        confirmed_msg.point.z = confirmed_entry['point'][2]
        self.confirmed_pub.publish(confirmed_msg)

        self.recent_points.clear()
        self.enter_wait_state()

        self.get_logger().info(
            '确定抓取目标: '
            f'frame={confirmed_msg.header.frame_id}, '
            f'point=[{confirmed_msg.point.x:.3f}, {confirmed_msg.point.y:.3f}, {confirmed_msg.point.z:.3f}]'
        )

    def is_window_stable(self):
        points = [item['point'] for item in self.recent_points]
        count = float(len(points))
        centroid = (
            sum(p[0] for p in points) / count,
            sum(p[1] for p in points) / count,
            sum(p[2] for p in points) / count,
        )

        stability_radius = float(self.get_parameter('stability_radius_m').value)
        for point in points:
            if self.distance(point, centroid) > stability_radius:
                return False
        return True

    def enter_wait_state(self):
        self.waiting_for_execution_complete = True
        self.wait_start_time_ns = self.get_clock().now().nanoseconds
        self.seen_execution_busy_true = self.last_execution_busy
        self.get_logger().info('等待机械臂执行完成后再允许下一次目标发布。')

    def execution_busy_callback(self, msg):
        busy = bool(msg.data)
        was_busy = self.last_execution_busy
        self.last_execution_busy = busy

        if not self.waiting_for_execution_complete:
            return

        if busy:
            self.seen_execution_busy_true = True
            return

        if was_busy or self.seen_execution_busy_true:
            self.reset_wait_state('机械臂执行已结束。')

    def check_wait_timeout(self):
        if not self.waiting_for_execution_complete:
            return

        timeout_sec = float(self.get_parameter('wait_completion_timeout_sec').value)
        if timeout_sec <= 0.0:
            return

        elapsed_sec = (
            self.get_clock().now().nanoseconds - self.wait_start_time_ns
        ) / 1e9
        if elapsed_sec < timeout_sec:
            return

        self.get_logger().warn(
            f'等待机械臂执行完成超时（{elapsed_sec:.1f}s >= {timeout_sec:.1f}s），已自动解锁。'
        )
        self.reset_wait_state('执行完成超时自动解锁。')

    def reset_wait_state(self, reason):
        self.waiting_for_execution_complete = False
        self.wait_start_time_ns = 0
        self.seen_execution_busy_true = False
        self.recent_points.clear()
        self.get_logger().info(f'Reset confirmed target: {reason}')

    @staticmethod
    def distance(point_a, point_b):
        return math.sqrt(
            (point_a[0] - point_b[0]) ** 2 +
            (point_a[1] - point_b[1]) ** 2 +
            (point_a[2] - point_b[2]) ** 2
        )


def main(args=None):
    rclpy.init(args=args)
    node = PepperBingConfirmer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
