import ast
import os
import sys
import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image


class PepperDetectorGPU(Node):
    def __init__(self):
        super().__init__('pepper_detector_gpu')
        self.bridge = CvBridge()

        self.declare_parameter('model_path', self._resolve_default_model_path())
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_info_yaml', '')
        self.declare_parameter('debug_image_topic', '/detect/image')
        self.declare_parameter('all_points_topic', '/pepper/detected_points')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('depth_window_size', 5)
        self.declare_parameter('min_valid_depth_m', 0.15)
        self.declare_parameter('max_valid_depth_m', 1.50)
        self.declare_parameter('max_depth_age_sec', 0.20)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('rou_pose_topic', '/pepper/rou_pose')
        self.declare_parameter('bing_pose_topic', '/pepper/bing_pose')

        self.latest_depth = None
        self.latest_depth_header = None
        self.intrinsics = None
        self.intrinsics_from_yaml = False

        # 推理线程用的最新彩色帧（只保留最新一帧）
        self._pending_color_msg = None
        self._pending_lock = threading.Lock()

        self._initialize_intrinsics()
        self._load_session()

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(Image, self.get_parameter('color_topic').value, self._image_cb, sensor_qos)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self._depth_cb, sensor_qos)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value, self._caminfo_cb, sensor_qos
        )

        self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 10)
        self.points_pub = self.create_publisher(PoseArray, self.get_parameter('all_points_topic').value, 10)
        self.rou_pose_pub = self.create_publisher(PoseStamped, self.get_parameter('rou_pose_topic').value, 10)
        self.bing_pose_pub = self.create_publisher(PoseStamped, self.get_parameter('bing_pose_topic').value, 10)

        self._infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self._infer_thread.start()

        self.get_logger().info(
            f'pepper_detector_gpu ready. model={self.get_parameter("model_path").value}, '
            f'detect_class=rou,bing, providers={self.session.get_providers()}'
        )

    def _resolve_default_model_path(self):
        try:
            from ament_index_python.packages import get_package_share_directory
            p = Path(get_package_share_directory('realsense_bringup')) / 'config' / 'best.onnx'
            if p.is_file():
                return str(p)
        except Exception:
            pass
        return str(Path(__file__).resolve().parents[1] / 'config' / 'best.onnx')

    def _initialize_intrinsics(self):
        yaml_path = str(self.get_parameter('camera_info_yaml').value).strip()
        if not yaml_path:
            return
        intrinsics = self._load_intrinsics_from_yaml(yaml_path)
        if intrinsics is None:
            self.get_logger().warning(
                f'Failed to load intrinsics from "{yaml_path}", falling back to camera_info topic.'
            )
            return
        self.intrinsics = intrinsics
        self.intrinsics_from_yaml = True

    def _load_intrinsics_from_yaml(self, yaml_path):
        normalized = yaml_path.removeprefix('file://')
        candidates = [Path(normalized).expanduser()]
        if not candidates[0].is_absolute():
            candidates.append(Path(__file__).resolve().parents[1] / normalized)
        for p in candidates:
            if p.is_file():
                try:
                    with p.open('r', encoding='utf-8') as f:
                        data = yaml.safe_load(f) or {}
                    k = data.get('camera_matrix', {}).get('data', [])
                    if len(k) == 9:
                        return [float(v) for v in k]
                except Exception:
                    pass
        return None

    def _load_session(self):
        try:
            import onnxruntime as ort
        except ImportError:
            self._inject_perception_venv()
            import onnxruntime as ort

        model_path = Path(self.get_parameter('model_path').value).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'ONNX model not found: {model_path}')

        providers = [('CUDAExecutionProvider', {'device_id': 0}), 'CPUExecutionProvider']
        self.session = ort.InferenceSession(str(model_path), providers=providers)

        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        self.input_height = int(inp.shape[2])
        self.input_width = int(inp.shape[3])

        raw = self.session.get_modelmeta().custom_metadata_map.get('names', '{}')
        try:
            parsed = ast.literal_eval(raw)
            self.class_names = {int(k): v for k, v in parsed.items()} if isinstance(parsed, dict) else {}
        except (ValueError, SyntaxError):
            self.class_names = {}

    def _inject_perception_venv(self):
        env_val = os.environ.get('PERCEPTION_SITE_PACKAGES', '')
        candidates = [Path(env_val)] if env_val else []
        for parent in Path(__file__).resolve().parents:
            venv = parent / '.venv_perception'
            if venv.exists():
                candidates.extend(sorted(venv.glob('lib/python*/site-packages')))
        for p in candidates:
            s = str(p)
            if p.is_dir() and s not in sys.path:
                sys.path.insert(0, s)

    def _caminfo_cb(self, msg):
        if not self.intrinsics_from_yaml:
            self.intrinsics = msg.k

    def _depth_cb(self, msg):
        self.latest_depth_header = msg.header
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def _image_cb(self, msg):
        # 只存最新帧，推理线程自己取
        with self._pending_lock:
            self._pending_color_msg = msg

    def _infer_loop(self):
        while rclpy.ok():
            with self._pending_lock:
                msg = self._pending_color_msg
                self._pending_color_msg = None
            if msg is None or self.latest_depth is None or self.intrinsics is None:
                threading.Event().wait(0.005)
                continue

            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if not self._is_depth_fresh(msg.header):
                cv2.putText(frame, 'Depth frame too old', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                self._publish_points([], msg.header)
                self._publish_debug(frame, msg.header)
                cv2.imshow('pepper_detector_gpu', frame)
                cv2.waitKey(1)
                continue

            detections = self._run_inference(frame)

            rou_detections = [d for d in detections if d['class_name'] == 'rou']
            bing_detections = [d for d in detections if d['class_name'] == 'bing']

            valid_points = []
            best_rou_xyz = None
            best_rou_score = -1.0
            best_bing_xyz = None
            best_bing_score = -1.0

            for det in rou_detections:
                u = int(round((det['x1'] + det['x2']) * 0.5))
                v = int(round((det['y1'] + det['y2']) * 0.5))
                depth_m = self._lookup_depth(u, v)
                xyz = self._project_pixel_to_3d(u, v, depth_m) if depth_m is not None else None

                label_main = f"rou {det['score']:.2f}"
                if xyz is not None:
                    label_detail = f'cam=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})m'
                    valid_points.append(xyz)
                    if det['score'] > best_rou_score:
                        best_rou_score = det['score']
                        best_rou_xyz = xyz
                else:
                    label_detail = 'depth=invalid'

                self._draw_detection(frame, det, u, v, label_main, label_detail, xyz is not None, 'rou')

            for det in bing_detections:
                u = int(round((det['x1'] + det['x2']) * 0.5))
                v = int(round((det['y1'] + det['y2']) * 0.5))
                depth_m = self._lookup_depth(u, v)
                xyz = self._project_pixel_to_3d(u, v, depth_m) if depth_m is not None else None

                label_main = f"bing {det['score']:.2f}"
                if xyz is not None:
                    label_detail = f'cam=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})m'
                    valid_points.append(xyz)
                    if det['score'] > best_bing_score:
                        best_bing_score = det['score']
                        best_bing_xyz = xyz
                else:
                    label_detail = 'depth=invalid'

                self._draw_detection(frame, det, u, v, label_main, label_detail, xyz is not None, 'bing')

            self._publish_points(valid_points, msg.header)
            if best_rou_xyz is not None:
                self._publish_rou_pose(best_rou_xyz, msg.header)
            if best_bing_xyz is not None:
                self._publish_bing_pose(best_bing_xyz, msg.header)
            self._publish_debug(frame, msg.header)
            cv2.imshow('pepper_detector_gpu', frame)
            cv2.waitKey(1)

    def _run_inference(self, frame):
        input_tensor, ratio, pad = self._preprocess(frame)
        prediction = self.session.run(None, {self.input_name: input_tensor})[0]
        return self._postprocess(prediction, frame.shape[:2], ratio, pad)

    def _preprocess(self, frame):
        image, ratio, pad = self._letterbox(frame, (self.input_height, self.input_width))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.transpose(2, 0, 1)
        image = np.ascontiguousarray(image, dtype=np.float32) / 255.0
        return np.expand_dims(image, axis=0), ratio, pad

    def _postprocess(self, prediction, original_shape, ratio, pad):
        conf_threshold = float(self.get_parameter('confidence_threshold').value)
        iou_threshold = float(self.get_parameter('iou_threshold').value)

        pred = np.squeeze(prediction, axis=0).T
        boxes_xywh = pred[:, :4]
        class_scores = pred[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = np.max(class_scores, axis=1)

        valid = confidences >= conf_threshold
        if not np.any(valid):
            return []

        boxes_xywh = boxes_xywh[valid]
        confidences = confidences[valid]
        class_ids = class_ids[valid]

        boxes_xyxy = np.empty_like(boxes_xywh)
        boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] * 0.5
        boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] * 0.5
        boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] * 0.5
        boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] * 0.5

        pad_x, pad_y = pad
        boxes_xyxy[:, [0, 2]] = (boxes_xyxy[:, [0, 2]] - pad_x) / ratio
        boxes_xyxy[:, [1, 3]] = (boxes_xyxy[:, [1, 3]] - pad_y) / ratio

        height, width = original_shape
        boxes_xyxy[:, 0] = np.clip(boxes_xyxy[:, 0], 0, width - 1)
        boxes_xyxy[:, 2] = np.clip(boxes_xyxy[:, 2], 0, width - 1)
        boxes_xyxy[:, 1] = np.clip(boxes_xyxy[:, 1], 0, height - 1)
        boxes_xyxy[:, 3] = np.clip(boxes_xyxy[:, 3], 0, height - 1)

        nms_boxes = [
            [float(b[0]), float(b[1]), float(b[2] - b[0]), float(b[3] - b[1])]
            for b in boxes_xyxy
        ]
        indices = cv2.dnn.NMSBoxes(nms_boxes, confidences.tolist(), conf_threshold, iou_threshold)
        if len(indices) == 0:
            return []

        return [
            {
                'x1': float(boxes_xyxy[i, 0]),
                'y1': float(boxes_xyxy[i, 1]),
                'x2': float(boxes_xyxy[i, 2]),
                'y2': float(boxes_xyxy[i, 3]),
                'score': float(confidences[i]),
                'class_id': int(class_ids[i]),
                'class_name': self.class_names.get(int(class_ids[i]), str(int(class_ids[i]))),
            }
            for i in np.array(indices).reshape(-1)
        ]

    @staticmethod
    def _letterbox(image, new_shape, color=(114, 114, 114)):
        height, width = image.shape[:2]
        new_height, new_width = new_shape
        ratio = min(new_height / height, new_width / width)
        rw = int(round(width * ratio))
        rh = int(round(height * ratio))
        resized = cv2.resize(image, (rw, rh), interpolation=cv2.INTER_LINEAR)
        pad_w = new_width - rw
        pad_h = new_height - rh
        pad_left = int(round(pad_w / 2.0 - 0.1))
        pad_top = int(round(pad_h / 2.0 - 0.1))
        bordered = cv2.copyMakeBorder(
            resized,
            pad_top, new_height - rh - pad_top,
            pad_left, new_width - rw - pad_left,
            cv2.BORDER_CONSTANT, value=color,
        )
        return bordered, ratio, (pad_left, pad_top)

    def _lookup_depth(self, u, v):
        if self.latest_depth is None:
            return None
        half = max(0, int(self.get_parameter('depth_window_size').value) // 2)
        min_d = float(self.get_parameter('min_valid_depth_m').value)
        max_d = float(self.get_parameter('max_valid_depth_m').value)
        h, w = self.latest_depth.shape[:2]
        roi = self.latest_depth[max(0, v - half):min(h, v + half + 1), max(0, u - half):min(w, u + half + 1)]
        if roi.size == 0:
            return None
        values = (roi.astype(np.float32) / 1000.0) if roi.dtype == np.uint16 else roi.astype(np.float32)
        values = values[np.isfinite(values)]
        values = values[(values > 0.0) & (values >= min_d) & (values <= max_d)]
        return float(np.median(values)) if values.size > 0 else None

    def _is_depth_fresh(self, color_header):
        if self.latest_depth_header is None:
            return False
        max_age = float(self.get_parameter('max_depth_age_sec').value)
        if max_age <= 0.0:
            return True
        t_color = Time.from_msg(color_header.stamp)
        t_depth = Time.from_msg(self.latest_depth_header.stamp)
        if t_color.nanoseconds == 0 or t_depth.nanoseconds == 0:
            return True
        return abs((t_color - t_depth).nanoseconds) / 1e9 <= max_age

    def _project_pixel_to_3d(self, u, v, depth_m):
        fx, fy = self.intrinsics[0], self.intrinsics[4]
        cx, cy = self.intrinsics[2], self.intrinsics[5]
        return (u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m

    def _draw_detection(self, frame, det, u, v, label_main, label_detail, valid, class_name='rou'):
        x1, y1, x2, y2 = int(round(det['x1'])), int(round(det['y1'])), int(round(det['x2'])), int(round(det['y2']))
        if valid:
            if class_name == 'rou':
                box_color = (0, 220, 0)
                pt_color = (0, 255, 255)
            else:
                box_color = (255, 140, 0)
                pt_color = (255, 200, 0)
        else:
            box_color = (0, 0, 255)
            pt_color = (0, 165, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        cv2.circle(frame, (u, v), 4, pt_color, -1)
        line1_y = max(20, y1 - 12)
        line2_y = min(frame.shape[0] - 10, line1_y + 20)
        cv2.putText(frame, label_main, (x1, line1_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
        cv2.putText(frame, label_detail, (x1, line2_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

    def _publish_points(self, points, header):
        msg = PoseArray()
        msg.header = header
        for x, y, z in points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.points_pub.publish(msg)

    def _publish_rou_pose(self, xyz, header):
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        self.rou_pose_pub.publish(msg)

    def _publish_bing_pose(self, xyz, header):
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.w = 1.0
        self.bing_pose_pub.publish(msg)

    def _publish_debug(self, frame, header):
        if not self.get_parameter('publish_debug_image').value:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header = header
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PepperDetectorGPU()
    except Exception as exc:
        print(f'[pepper_detector_gpu] startup failed: {exc}')
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
