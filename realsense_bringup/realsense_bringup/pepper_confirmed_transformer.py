import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class PepperConfirmedTransformer(Node):
    def __init__(self):
        super().__init__('pepper_confirmed_transformer')

        self.declare_parameter('input_topic', '/pepper/confirmed_point_camera')
        self.declare_parameter('output_topic', '/pepper/confirmed_point_base')
        self.declare_parameter('grab_pose_topic', '/grab_pose')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('bing_child_frame', 'bing')
        self.declare_parameter('lookup_timeout_sec', 0.2)
        self.declare_parameter('target_z_offset_m', 0.0)
        self.declare_parameter('grab_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('publish_bing_tf', True)
        self.declare_parameter('tf_warning_interval_sec', 2.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.output_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 10
        )
        self.grab_pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('grab_pose_topic').value, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter('input_topic').value,
            self.point_callback,
            10,
        )

        self.last_tf_warning_time_ns = 0

        self.get_logger().info(
            'Pepper confirmed transformer ready. '
            f'input={self.get_parameter("input_topic").value}, '
            f'output={self.get_parameter("output_topic").value}, '
            f'grab_topic={self.get_parameter("grab_pose_topic").value}, '
            f'target_frame={self.get_parameter("target_frame").value}, '
            f'bing_child_frame={self.get_parameter("bing_child_frame").value}'
        )

    def point_callback(self, msg):
        if not msg.header.frame_id:
            self.get_logger().warn('Received confirmed point with empty frame_id.')
            return

        target_frame = self.get_parameter('target_frame').value
        timeout = Duration(seconds=float(self.get_parameter('lookup_timeout_sec').value))
        point_stamp = Time.from_msg(msg.header.stamp)
        lookup_time = point_stamp if point_stamp.nanoseconds != 0 else Time()

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                lookup_time,
                timeout=timeout,
            )
        except TransformException as exc:
            self.warn_tf_unavailable(msg.header.frame_id, target_frame, exc)
            return

        transformed = do_transform_point(msg, transform)
        transformed.header.frame_id = target_frame
        if point_stamp.nanoseconds != 0:
            transformed.header.stamp = msg.header.stamp
        else:
            transformed.header.stamp = self.get_clock().now().to_msg()

        self.output_pub.publish(transformed)

        if bool(self.get_parameter('publish_bing_tf').value):
            self.publish_bing_tf(transformed)

        self.publish_grab_pose(transformed)

    def publish_bing_tf(self, point_msg):
        child_frame = str(self.get_parameter('bing_child_frame').value).strip()
        if not child_frame:
            self.get_logger().warn('bing_child_frame is empty; skip TF publish.')
            return

        target_tf = TransformStamped()
        target_tf.header.stamp = point_msg.header.stamp
        target_tf.header.frame_id = point_msg.header.frame_id
        target_tf.child_frame_id = child_frame
        target_tf.transform.translation.x = point_msg.point.x
        target_tf.transform.translation.y = point_msg.point.y
        target_tf.transform.translation.z = point_msg.point.z
        target_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(target_tf)

    def publish_grab_pose(self, point_msg):
        qx, qy, qz, qw = self.parse_orientation()
        z_offset = float(self.get_parameter('target_z_offset_m').value)

        grab_pose = PoseStamped()
        grab_pose.header = point_msg.header
        grab_pose.pose.position.x = point_msg.point.x
        grab_pose.pose.position.y = point_msg.point.y
        grab_pose.pose.position.z = point_msg.point.z + z_offset
        grab_pose.pose.orientation.x = qx
        grab_pose.pose.orientation.y = qy
        grab_pose.pose.orientation.z = qz
        grab_pose.pose.orientation.w = qw
        self.grab_pose_pub.publish(grab_pose)

        self.get_logger().info(
            'Published confirmed /grab_pose: '
            f'[{grab_pose.pose.position.x:.3f}, '
            f'{grab_pose.pose.position.y:.3f}, '
            f'{grab_pose.pose.position.z:.3f}] '
            f'in {grab_pose.header.frame_id}'
        )

    def parse_orientation(self):
        value = self.get_parameter('grab_orientation_xyzw').value
        if hasattr(value, '__len__') and len(value) == 4:
            try:
                qx, qy, qz, qw = [float(v) for v in value]
                norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
                if norm > 1e-8:
                    return qx / norm, qy / norm, qz / norm, qw / norm
            except (TypeError, ValueError):
                pass
        return 0.0, 0.0, 0.0, 1.0

    def warn_tf_unavailable(self, source_frame, target_frame, exc):
        interval_sec = max(0.0, float(self.get_parameter('tf_warning_interval_sec').value))
        now_ns = self.get_clock().now().nanoseconds
        if interval_sec > 0.0:
            elapsed_sec = (now_ns - self.last_tf_warning_time_ns) / 1e9
            if self.last_tf_warning_time_ns != 0 and elapsed_sec < interval_sec:
                return
        self.last_tf_warning_time_ns = now_ns
        self.get_logger().warn(
            f'Cannot transform confirmed point {source_frame} -> {target_frame}: {exc}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PepperConfirmedTransformer()
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
