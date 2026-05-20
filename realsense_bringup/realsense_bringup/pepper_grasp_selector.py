import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class PepperGraspSelector(Node):
    def __init__(self):
        super().__init__('pepper_grasp_selector')

        self.declare_parameter('input_points_topic', '/pepper/detected_points')
        self.declare_parameter('grab_pose_topic', '/grab_pose')
        self.declare_parameter('selected_point_topic', '/pepper/selected_point_base')
        self.declare_parameter('marker_topic', '/pepper/detected_markers')
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('lookup_timeout_sec', 0.2)
        self.declare_parameter('tf_warning_interval_sec', 2.0)

        self.declare_parameter('enable_auto_grab', True)
        self.declare_parameter('min_command_interval_sec', 6.0)
        self.declare_parameter('min_command_distance_m', 0.03)
        self.declare_parameter('required_stable_frames', 3)
        self.declare_parameter('stability_distance_m', 0.02)

        self.declare_parameter('workspace_min_x', 0.08)
        self.declare_parameter('workspace_max_x', 0.48)
        self.declare_parameter('workspace_min_y', -0.30)
        self.declare_parameter('workspace_max_y', 0.30)
        self.declare_parameter('workspace_min_z', 0.02)
        self.declare_parameter('workspace_max_z', 0.50)
        self.declare_parameter('workspace_min_radius', 0.10)
        self.declare_parameter('workspace_max_radius', 0.60)

        self.declare_parameter('target_z_offset_m', 0.0)
        self.declare_parameter('grab_orientation_xyzw', [0.0, 0.0, 0.0, 1.0])

        # Sphere diameter default = 4 cm (pepper size).
        self.declare_parameter('candidate_marker_size_m', 0.04)
        self.declare_parameter('selected_marker_size_m', 0.045)
        self.declare_parameter('marker_lifetime_sec', 0.6)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grab_pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter('grab_pose_topic').value, 10
        )
        self.selected_point_pub = self.create_publisher(
            PointStamped, self.get_parameter('selected_point_topic').value, 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10
        )

        self.create_subscription(
            PoseArray,
            self.get_parameter('input_points_topic').value,
            self.points_callback,
            10,
        )

        self.last_command_point = None
        self.last_command_time = None
        self.stable_candidate_point = None
        self.stable_count = 0
        self.last_tf_warning_time_ns = 0

        self.get_logger().info(
            'Pepper grasp selector ready. '
            f'input={self.get_parameter("input_points_topic").value}, '
            f'grab_topic={self.get_parameter("grab_pose_topic").value}, '
            f'target_frame={self.get_parameter("target_frame").value}'
        )

    def points_callback(self, msg):
        candidates = self.transform_candidates(msg)
        if candidates is None:
            return

        selected = self.pick_nearest_reachable(candidates)
        self.publish_markers(candidates, selected)

        if selected is None:
            self.reset_stability()
            return

        self.publish_selected_point(selected, msg.header.stamp)

        if not bool(self.get_parameter('enable_auto_grab').value):
            return
        if not self.update_stability(selected):
            return
        if not self.should_publish_grab(selected):
            return

        self.publish_grab_pose(selected, msg.header.stamp)

    def transform_candidates(self, msg):
        target_frame = self.get_parameter('target_frame').value
        timeout = Duration(seconds=float(self.get_parameter('lookup_timeout_sec').value))
        point_stamp = Time.from_msg(msg.header.stamp)
        lookup_time = point_stamp if point_stamp.nanoseconds != 0 else Time()

        if not msg.header.frame_id:
            self.get_logger().warn('Received candidate points with empty frame_id.')
            return []

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                msg.header.frame_id,
                lookup_time,
                timeout=timeout,
            )
        except TransformException as exc:
            self.warn_tf_unavailable(msg.header.frame_id, target_frame, exc)
            return None

        transformed = []
        for pose in msg.poses:
            point = PointStamped()
            point.header = msg.header
            point.point = pose.position
            point_base = do_transform_point(point, transform)

            x = float(point_base.point.x)
            y = float(point_base.point.y)
            z = float(point_base.point.z)
            reachable = self.is_reachable(x, y, z)
            distance = math.sqrt(x * x + y * y + z * z)

            transformed.append(
                {
                    'x': x,
                    'y': y,
                    'z': z,
                    'distance': distance,
                    'reachable': reachable,
                }
            )

        return transformed

    def is_reachable(self, x, y, z):
        min_x = float(self.get_parameter('workspace_min_x').value)
        max_x = float(self.get_parameter('workspace_max_x').value)
        min_y = float(self.get_parameter('workspace_min_y').value)
        max_y = float(self.get_parameter('workspace_max_y').value)
        min_z = float(self.get_parameter('workspace_min_z').value)
        max_z = float(self.get_parameter('workspace_max_z').value)
        min_r = float(self.get_parameter('workspace_min_radius').value)
        max_r = float(self.get_parameter('workspace_max_radius').value)

        radius = math.sqrt(x * x + y * y + z * z)
        return (
            min_x <= x <= max_x and
            min_y <= y <= max_y and
            min_z <= z <= max_z and
            min_r <= radius <= max_r
        )

    @staticmethod
    def pick_nearest_reachable(candidates):
        reachable = [item for item in candidates if item['reachable']]
        if not reachable:
            return None
        return min(reachable, key=lambda item: item['distance'])

    def reset_stability(self):
        self.stable_candidate_point = None
        self.stable_count = 0

    def update_stability(self, selected):
        tolerance = float(self.get_parameter('stability_distance_m').value)
        required_frames = max(1, int(self.get_parameter('required_stable_frames').value))
        point = (selected['x'], selected['y'], selected['z'])

        if self.stable_candidate_point is None:
            self.stable_candidate_point = point
            self.stable_count = 1
            return self.stable_count >= required_frames

        distance = math.sqrt(
            (point[0] - self.stable_candidate_point[0]) ** 2 +
            (point[1] - self.stable_candidate_point[1]) ** 2 +
            (point[2] - self.stable_candidate_point[2]) ** 2
        )

        if distance <= tolerance:
            self.stable_count += 1
        else:
            self.stable_candidate_point = point
            self.stable_count = 1

        return self.stable_count >= required_frames

    def should_publish_grab(self, selected):
        now = self.get_clock().now()
        min_interval = float(self.get_parameter('min_command_interval_sec').value)
        min_distance = float(self.get_parameter('min_command_distance_m').value)
        point = (selected['x'], selected['y'], selected['z'])

        if self.last_command_time is not None:
            elapsed = (now - self.last_command_time).nanoseconds / 1e9
            if elapsed < min_interval:
                return False

        if self.last_command_point is None:
            return True

        move = math.sqrt(
            (point[0] - self.last_command_point[0]) ** 2 +
            (point[1] - self.last_command_point[1]) ** 2 +
            (point[2] - self.last_command_point[2]) ** 2
        )
        return move >= min_distance

    def publish_grab_pose(self, selected, source_stamp):
        z_offset = float(self.get_parameter('target_z_offset_m').value)
        target_frame = self.get_parameter('target_frame').value
        qx, qy, qz, qw = self.parse_orientation()

        grab_pose = PoseStamped()
        grab_pose.header.frame_id = target_frame
        if Time.from_msg(source_stamp).nanoseconds != 0:
            grab_pose.header.stamp = source_stamp
        else:
            grab_pose.header.stamp = self.get_clock().now().to_msg()

        grab_pose.pose.position.x = selected['x']
        grab_pose.pose.position.y = selected['y']
        grab_pose.pose.position.z = selected['z'] + z_offset
        grab_pose.pose.orientation.x = qx
        grab_pose.pose.orientation.y = qy
        grab_pose.pose.orientation.z = qz
        grab_pose.pose.orientation.w = qw
        self.grab_pose_pub.publish(grab_pose)

        self.last_command_time = self.get_clock().now()
        self.last_command_point = (
            grab_pose.pose.position.x,
            grab_pose.pose.position.y,
            grab_pose.pose.position.z,
        )

        self.get_logger().info(
            'Published /grab_pose: '
            f'[{grab_pose.pose.position.x:.3f}, '
            f'{grab_pose.pose.position.y:.3f}, '
            f'{grab_pose.pose.position.z:.3f}]'
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

    def publish_selected_point(self, selected, source_stamp):
        msg = PointStamped()
        msg.header.frame_id = self.get_parameter('target_frame').value
        if Time.from_msg(source_stamp).nanoseconds != 0:
            msg.header.stamp = source_stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = selected['x']
        msg.point.y = selected['y']
        msg.point.z = selected['z']
        self.selected_point_pub.publish(msg)

    def publish_markers(self, candidates, selected):
        target_frame = self.get_parameter('target_frame').value
        candidate_size = float(self.get_parameter('candidate_marker_size_m').value)
        selected_size = float(self.get_parameter('selected_marker_size_m').value)
        lifetime = float(self.get_parameter('marker_lifetime_sec').value)
        stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()

        clear_marker = Marker()
        clear_marker.header.frame_id = target_frame
        clear_marker.header.stamp = stamp
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for index, item in enumerate(candidates):
            marker = Marker()
            marker.header.frame_id = target_frame
            marker.header.stamp = stamp
            marker.ns = 'pepper_candidates'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = item['x']
            marker.pose.position.y = item['y']
            marker.pose.position.z = item['z']
            marker.pose.orientation.w = 1.0
            marker.scale.x = candidate_size
            marker.scale.y = candidate_size
            marker.scale.z = candidate_size

            if item['reachable']:
                marker.color.r = 0.15
                marker.color.g = 0.85
                marker.color.b = 0.20
            else:
                marker.color.r = 0.90
                marker.color.g = 0.15
                marker.color.b = 0.15
            marker.color.a = 0.80
            self.set_marker_lifetime(marker, lifetime)
            marker_array.markers.append(marker)

        if selected is not None:
            selected_marker = Marker()
            selected_marker.header.frame_id = target_frame
            selected_marker.header.stamp = stamp
            selected_marker.ns = 'pepper_selected'
            selected_marker.id = 0
            selected_marker.type = Marker.SPHERE
            selected_marker.action = Marker.ADD
            selected_marker.pose.position.x = selected['x']
            selected_marker.pose.position.y = selected['y']
            selected_marker.pose.position.z = selected['z']
            selected_marker.pose.orientation.w = 1.0
            selected_marker.scale.x = selected_size
            selected_marker.scale.y = selected_size
            selected_marker.scale.z = selected_size
            selected_marker.color.r = 1.0
            selected_marker.color.g = 0.85
            selected_marker.color.b = 0.05
            selected_marker.color.a = 0.95
            self.set_marker_lifetime(selected_marker, lifetime)
            marker_array.markers.append(selected_marker)

        self.marker_pub.publish(marker_array)

    @staticmethod
    def set_marker_lifetime(marker, seconds):
        secs = int(max(0.0, seconds))
        nsecs = int((max(0.0, seconds) - secs) * 1e9)
        marker.lifetime.sec = secs
        marker.lifetime.nanosec = nsecs

    def warn_tf_unavailable(self, source_frame, target_frame, exc):
        interval_sec = max(0.0, float(self.get_parameter('tf_warning_interval_sec').value))
        now_ns = self.get_clock().now().nanoseconds
        if interval_sec > 0.0:
            elapsed_sec = (now_ns - self.last_tf_warning_time_ns) / 1e9
            if self.last_tf_warning_time_ns != 0 and elapsed_sec < interval_sec:
                return
        self.last_tf_warning_time_ns = now_ns
        self.get_logger().warn(
            'Cannot transform '
            f'{source_frame} -> {target_frame}: {exc}. '
            'If target_frame is base_link, start robot_state_publisher/MoveIt first.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = PepperGraspSelector()
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
