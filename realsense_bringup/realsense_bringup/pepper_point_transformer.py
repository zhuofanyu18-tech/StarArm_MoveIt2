from geometry_msgs.msg import PointStamped, TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


class PepperPointTransformer(Node):
    def __init__(self):
        super().__init__('pepper_point_transformer')

        self.declare_parameter('input_topic', '/pepper/target_point')
        self.declare_parameter('output_topic', '/pepper/target_point_base')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('lookup_timeout_sec', 0.2)
        self.declare_parameter('publish_target_tf', True)
        self.declare_parameter('target_child_frame', 'pepper_target')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.output_pub = self.create_publisher(
            PointStamped, self.get_parameter('output_topic').value, 10
        )
        self.create_subscription(
            PointStamped,
            self.get_parameter('input_topic').value,
            self.point_callback,
            10,
        )

        self.get_logger().info(
            f'Pepper point transformer ready. '
            f'{self.get_parameter("input_topic").value} -> '
            f'{self.get_parameter("output_topic").value} '
            f'in frame {self.get_parameter("target_frame").value}'
        )

    def point_callback(self, msg):
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
            self.get_logger().warn(
                f'Cannot transform {msg.header.frame_id} -> {target_frame}: {exc}'
            )
            return

        transformed = do_transform_point(msg, transform)
        transformed.header.frame_id = target_frame
        if point_stamp.nanoseconds != 0:
            transformed.header.stamp = msg.header.stamp
        self.output_pub.publish(transformed)

        if self.get_parameter('publish_target_tf').value:
            self.publish_target_tf(transformed)

    def publish_target_tf(self, point_msg):
        target_tf = TransformStamped()
        target_tf.header.stamp = point_msg.header.stamp
        target_tf.header.frame_id = point_msg.header.frame_id
        target_tf.child_frame_id = self.get_parameter('target_child_frame').value
        target_tf.transform.translation.x = point_msg.point.x
        target_tf.transform.translation.y = point_msg.point.y
        target_tf.transform.translation.z = point_msg.point.z
        target_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(target_tf)


def main(args=None):
    rclpy.init(args=args)
    node = PepperPointTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
