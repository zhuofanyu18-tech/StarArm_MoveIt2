import ast
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener


class pepperDetector3D(Node):
    """
    pepperDetector3D 节点：基于 YOLO ONNX 模型的 3D 辣椒检测器
    
    功能说明：
    1. 订阅 RGB 彩色图像、深度图像和相机内参话题
    2. 使用 ONNX Runtime 运行 YOLO 目标检测模型
    3. 结合深度信息计算检测目标的世界坐标 (x, y, z)
    4. 发布带标注的调试图像和目标 3D 点云坐标
    
    检测流程：
    彩色图像 -> YOLO推理 -> 检测框 -> 结合深度图像 -> 3D坐标投影 -> 发布结果
    """
    
    def __init__(self):
        super().__init__('pepper_detector')
        self.bridge = CvBridge()

        # YOLO onnx模型的路径
        # 声明可动态配置的参数
        # model_path: YOLO ONNX 模型文件路径
        # color_topic/depth_topic/camera_info_topic: ROS 话题名称
        # confidence_threshold: 目标检测置信度阈值
        # iou_threshold: NMS 非极大值抑制的 IOU 阈值
        # depth_window_size: 深度查找窗口大小，用于平滑深度值
        # min_valid_depth_m/max_valid_depth_m: 有效深度范围（米）
        # max_depth_age_sec: 深度图像最大延迟，超过则视为过期
        # stem_class_name/fruit_class_name: 茎和果实的类别名称
        self.declare_parameter('model_path', self._resolve_default_model_path())
        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_info_yaml', '')
        self.declare_parameter('debug_image_topic', '/detect/image')
        self.declare_parameter('target_point_topic', '/pepper/target_point')
        self.declare_parameter('all_points_topic', '/pepper/detected_points')
        self.declare_parameter('overlay_target_frame', 'base_link')
        self.declare_parameter('overlay_tf_timeout_sec', 0.20)
        self.declare_parameter('overlay_tf_warning_interval_sec', 2.0)
        self.declare_parameter('show_position_overlay', True)
        self.declare_parameter('position_log_interval_sec', 1.0)
        self.declare_parameter('execution_busy_topic', '/grab/execution_busy')
        self.declare_parameter('confirm_point_topic', '/pepper/confirmed_point_camera')
        self.declare_parameter('target_publish_hold_timeout_sec', 120.0)
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('depth_window_size', 5)
        self.declare_parameter('min_valid_depth_m', 0.15)
        self.declare_parameter('max_valid_depth_m', 1.50)
        self.declare_parameter('max_depth_age_sec', 0.20)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('stem_class_name', 'bing')
        self.declare_parameter('fruit_class_name', 'rou')

        # 注册参数回调函数，支持运行时动态修改参数
        self.add_on_set_parameters_callback(self.parameter_callback)

        # 加载 ONNX Runtime 和模型会话
        self.ort = self._load_onnxruntime()
        self.session = self._load_session()
        
        # 获取模型输入层的名称和形状
        # input_name: 输入张量的名称（如 "images"）
        # input_height/input_width: 模型要求的输入图像尺寸（通常 YOLO 为 640x640）
        self.input_name = self.session.get_inputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        self.input_height = int(input_shape[2])
        self.input_width = int(input_shape[3])
        
        # 加载类别名称映射（从模型的 metadata 中获取，如 {0: 'bing', 1: 'rou'}）
        self.class_names = self._load_class_names()

        # 存储最新的深度图像和相机内参
        self.latest_depth = None           # 最近接收的深度图像（CV2 格式）
        self.latest_depth_header = None    # 深度图像的时间戳头信息
        self.intrinsics = None             # 相机内参矩阵 k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.intrinsics_from_yaml = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_position_log_time_ns = 0
        self.last_overlay_tf_warning_time_ns = 0
        self.last_overlay_tf_fallback_log_time_ns = 0
        self.target_publish_hold = False
        self.target_publish_hold_start_ns = 0
        self.last_execution_busy = False
        self.execution_busy_seen_true = False

        # 创建订阅者和发布者
        # 使用 SENSOR_DATA QoS 配置，确保实时性
        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value

        # 优先尝试从外部标定文件加载内参
        self._initialize_intrinsics()

        # 订阅彩色图像、深度图像和相机内参话题
        self.create_subscription(Image, self.get_parameter('color_topic').value, self.image_callback, sensor_qos)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_callback, sensor_qos)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.camera_info_callback, sensor_qos)
        self.create_subscription(
            PointStamped,
            self.get_parameter('confirm_point_topic').value,
            self.confirm_point_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('execution_busy_topic').value,
            self.execution_busy_callback,
            10,
        )

        # 发布带检测标注的调试图像
        self.debug_image_pub = self.create_publisher(
            Image, self.get_parameter('debug_image_topic').value, 10
        )
        # 发布检测到的最佳目标点的 3D 坐标
        self.target_point_pub = self.create_publisher(
            PointStamped, self.get_parameter('target_point_topic').value, 10
        )
        # 发布所有有效检测点（3D，原始相机坐标系）
        self.all_points_pub = self.create_publisher(
            PoseArray, self.get_parameter('all_points_topic').value, 10
        )

        self.get_logger().info(
            f'Pepper detector ready. model={self.get_parameter("model_path").value}, '
            f'classes={self.class_names}, '
            f'intrinsics_source={"yaml" if self.intrinsics_from_yaml else "camera_info_topic"}, '
            f'overlay_frame={self.get_parameter("overlay_target_frame").value}, '
            f'show_position_overlay={self.get_parameter("show_position_overlay").value}, '
            f'position_log_interval_sec={float(self.get_parameter("position_log_interval_sec").value):.2f}, '
            f'execution_busy_topic={self.get_parameter("execution_busy_topic").value}, '
            f'confirm_point_topic={self.get_parameter("confirm_point_topic").value}'
        )

    def _initialize_intrinsics(self):
        """
        尝试从 camera_info_yaml 参数指定的标定文件加载相机内参

        成功时将锁定使用 YAML 内参，不再被 camera_info 话题覆盖。
        """
        yaml_path = str(self.get_parameter('camera_info_yaml').value).strip()
        if not yaml_path:
            return

        intrinsics, resolved_path = self._load_intrinsics_from_yaml(yaml_path)
        if intrinsics is None:
            self.get_logger().warning(
                f'Failed to load camera intrinsics from yaml "{yaml_path}". '
                'Falling back to camera_info topic.'
            )
            return

        self.intrinsics = intrinsics
        self.intrinsics_from_yaml = True
        self.get_logger().info(f'Loaded camera intrinsics from yaml: {resolved_path}')

    def _load_intrinsics_from_yaml(self, yaml_path):
        """
        从 ROS camera calibration YAML 中读取 camera_matrix.data (9 个元素)
        """
        normalized = yaml_path
        if normalized.startswith('file://'):
            normalized = normalized[7:]

        candidate_paths = [Path(normalized).expanduser()]
        if not candidate_paths[0].is_absolute():
            package_root = Path(__file__).resolve().parents[1]
            candidate_paths.append(package_root / normalized)

        resolved_path = None
        for candidate in candidate_paths:
            if candidate.is_file():
                resolved_path = candidate
                break

        if resolved_path is None:
            return None, None

        try:
            with resolved_path.open('r', encoding='utf-8') as handle:
                data = yaml.safe_load(handle) or {}
            camera_matrix = data.get('camera_matrix', {})
            k_data = camera_matrix.get('data', [])
            if len(k_data) != 9:
                return None, None
            intrinsics = [float(value) for value in k_data]
            return intrinsics, str(resolved_path)
        except Exception:
            return None, None

    def _resolve_default_model_path(self):
        """
        解析默认的 ONNX 模型路径
        
        搜索策略（按优先级）：
        1. 尝试从 ament_index_python 获取 realsense_bringup 包路径
        2. 使用脚本所在目录的相对路径查找
        3. 返回第一个候选路径（即使文件不存在）
        """
        candidate_paths = []

        try:
            # 尝试获取 ROS2 包的共享目录
            from ament_index_python.packages import get_package_share_directory

            candidate_paths.append(
                Path(get_package_share_directory('realsense_bringup')) / 'config' / 'best.onnx'
            )
        except Exception:
            pass

        # 回退到脚本目录的相对路径
        package_root = Path(__file__).resolve().parents[1]
        candidate_paths.append(package_root / 'config' / 'best.onnx')

        # 返回第一个找到的路径
        for candidate in candidate_paths:
            if candidate.is_file():
                return str(candidate)

        return str(candidate_paths[0])

    def _load_onnxruntime(self):
        """
        加载 ONNX Runtime 库
        
        尝试策略：
        1. 直接导入 onnxruntime
        2. 失败后尝试注入自定义感知虚拟环境路径
        3. 再失败则抛出运行时错误
        """
        try:
            import onnxruntime as ort # pyright: ignore[reportMissingImports]
        except ImportError:
            # 尝试从自定义虚拟环境加载
            self._inject_perception_site_packages()
            try:
                import onnxruntime as ort # pyright: ignore[reportMissingImports]
            except ImportError as exc:
                raise RuntimeError(
                    'onnxruntime is not installed in the current Python environment. '
                    'Activate the perception venv before running this node.'
                ) from exc
        return ort

    def _inject_perception_site_packages(self):
        """
        注入感知虚拟环境的 site-packages 到 sys.path
        
        搜索机制：
        1. 检查环境变量 PERCEPTION_SITE_PACKAGES
        2. 查找脚本父目录下的 .venv_perception 虚拟环境
        3. 找到 lib/python*/site-packages 目录并插入到 sys.path 首位
        """
        candidate_paths = []

        # 优先使用环境变量指定的路径
        env_value = os.environ.get('PERCEPTION_SITE_PACKAGES', '')
        env_path = Path(env_value) if env_value else None
        if env_path:
            candidate_paths.append(env_path)

        # 自动搜索 .venv_perception 虚拟环境
        for parent in Path(__file__).resolve().parents:
            venv_root = parent / '.venv_perception'
            if not venv_root.exists():
                continue
            # 查找 lib/python*/site-packages 路径
            for site_pkg in sorted(venv_root.glob('lib/python*/site-packages')):
                candidate_paths.append(site_pkg)

        # 将找到的路径插入 sys.path 首位
        for candidate in candidate_paths:
            candidate_str = str(candidate)
            if candidate.is_dir() and candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)

    def _load_session(self):
        """
        加载 ONNX 模型会话
        
        使用 CPU 执行提供者加载模型，支持动态输入尺寸
        """
        model_path = Path(self.get_parameter('model_path').value).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f'ONNX model not found: {model_path}')
        return self.ort.InferenceSession(str(model_path), providers=['CPUExecutionProvider'])

    def _load_class_names(self):
        """
        从模型的 metadata 中加载类别名称映射
        
        metadata 中通常存储为 JSON 字符串格式，如：
        '{"0": "bing", "1": "rou"}'
        
        返回: {类别ID: 类别名称} 的字典
        """
        metadata = self.session.get_modelmeta().custom_metadata_map
        raw_names = metadata.get('names', '{}')
        try:
            # 使用 ast.literal_eval 安全解析字典字符串
            parsed = ast.literal_eval(raw_names)
            if isinstance(parsed, dict):
                # 确保 key 是整数类型
                return {int(key): value for key, value in parsed.items()}
        except (ValueError, SyntaxError):
            pass
        return {}

    def parameter_callback(self, params):
        """
        参数动态更新回调函数
        
        当通过 ROS2 参数服务器修改参数时，会触发此回调
        返回 SetParametersResult 表示参数更新是否成功
        """
        for param in params:
            self.get_logger().info(f'Parameter updated: {param.name} = {param.value}')
        return SetParametersResult(successful=True)

    def camera_info_callback(self, msg):
        """
        相机内参回调函数
        
        相机内参矩阵 K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        - fx, fy: 焦距（像素）
        - cx, cy: 主点坐标（像素）
        
        用于将 2D 像素坐标转换为 3D 世界坐标
        """
        if self.intrinsics_from_yaml:
            return
        self.intrinsics = msg.k

    def depth_callback(self, msg):
        """
        深度图像回调函数
        
        将 ROS 深度图像消息转换为 CV2 格式并存储
        深度图像用于计算检测目标的实际距离
        """
        self.latest_depth_header = msg.header
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        """
        彩色图像主处理回调函数
        
        处理流程：
        1. 检查深度图像和内参是否可用
        2. 检查深度图像是否过期（延迟过大）
        3. 运行 YOLO 目标检测
        4. 对每个检测结果：
           - 计算检测框中心点
           - 查找对应位置的深度值
           - 投影到 3D 世界坐标系
        5. 筛选有效的茎类目标
        6. 选择置信度最高的作为输出目标
        7. 发布调试图像
        
        关键设计：
        - 使用 latest_depth 而非同步接收，保证实时性
        - 深度延迟检查避免使用过期数据
        """
        self.check_target_publish_hold_timeout()

        if self.latest_depth is None or self.intrinsics is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 检查深度图像是否过期
        if not self.is_depth_fresh(msg.header):
            cv2.putText(
                frame,
                'Depth frame too old',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )
            overlay_transform = self.lookup_overlay_transform(msg.header)
            cam_in_base = self.extract_camera_position(overlay_transform)
            self.draw_position_overlay(
                frame,
                cam_in_base=cam_in_base,
                bing_in_camera=None,
                bing_in_base=None,
            )
            self.log_position_status(
                cam_in_base=cam_in_base,
                bing_in_camera=None,
                bing_in_base=None,
            )
            self.publish_all_points([], msg.header)
            self.publish_debug(frame, msg.header)
            return

        # 运行 YOLO 目标检测
        detections = self.run_inference(frame)
        if not detections:
            overlay_transform = self.lookup_overlay_transform(msg.header)
            cam_in_base = self.extract_camera_position(overlay_transform)
            self.draw_position_overlay(
                frame,
                cam_in_base=cam_in_base,
                bing_in_camera=None,
                bing_in_base=None,
            )
            self.log_position_status(
                cam_in_base=cam_in_base,
                bing_in_camera=None,
                bing_in_base=None,
            )
            self.publish_all_points([], msg.header)
            self.publish_debug(frame, msg.header)
            return

        # 处理每个检测结果，计算 3D 坐标
        overlay_transform = self.lookup_overlay_transform(msg.header)
        annotated = []
        for detection in detections:
            # 计算检测框中心点 (u, v)
            u = int(round((detection['x1'] + detection['x2']) * 0.5))
            v = int(round((detection['y1'] + detection['y2']) * 0.5))
            
            # 查找中心点的深度值
            depth_m = self.lookup_depth(u, v)

            xyz = None
            highlight = False
            label_main = f"{detection['class_name']} {detection['score']:.2f}"
            label_detail = 'depth=invalid'
            if depth_m is not None:
                # 将像素坐标 + 深度转换为 3D 坐标
                xyz = self.project_pixel_to_3d(u, v, depth_m)
                if overlay_transform is not None:
                    base_x, base_y, base_z = self.transform_point_to_overlay_frame(
                        xyz[0], xyz[1], xyz[2], msg.header, overlay_transform
                    )
                    radius_m = math.sqrt(base_x * base_x + base_y * base_y + base_z * base_z)
                    label_detail = (
                        f'base=({base_x:.3f},{base_y:.3f},{base_z:.3f})m '
                        f'd={radius_m:.3f}m'
                    )
                    highlight = True
                else:
                    label_detail = 'base=TF_FAIL'

            # 绘制检测框和标注
            self.draw_detection(
                frame,
                detection,
                u,
                v,
                label_main,
                label_detail,
                highlight=highlight,
            )
            annotated.append(
                {
                    'detection': detection,
                    'u': u,
                    'v': v,
                    'xyz': xyz,
                }
            )

        # 发布所有有效检测点（用于后续“最近可达目标”选择）
        valid_targets = [item for item in annotated if item['xyz'] is not None]
        self.publish_all_points(valid_targets, msg.header)

        # 兼容旧链路：保留“单目标点”输出（默认仍用茎类）
        stem_class_name = self.get_parameter('stem_class_name').value.strip()
        valid_stem_targets = [
            item for item in valid_targets
            if item['detection']['class_name'] == stem_class_name
        ]

        # 选择置信度最高的有效茎点（bing）作为抓取目标
        bing_in_camera = None
        bing_in_base = None
        if valid_stem_targets:
            best_bing = max(valid_stem_targets, key=lambda item: item['detection']['score'])
            x, y, z = best_bing['xyz']
            bing_in_camera = (x, y, z)
            if not self.target_publish_hold:
                self.publish_target_point(x, y, z, msg.header)
            if overlay_transform is not None:
                bing_in_base = self.transform_point_to_overlay_frame(
                    x, y, z, msg.header, overlay_transform
                )

        cam_in_base = self.extract_camera_position(overlay_transform)
        self.draw_position_overlay(
            frame,
            cam_in_base=cam_in_base,
            bing_in_camera=bing_in_camera,
            bing_in_base=bing_in_base,
        )
        self.log_position_status(
            cam_in_base=cam_in_base,
            bing_in_camera=bing_in_camera,
            bing_in_base=bing_in_base,
        )

        self.publish_debug(frame, msg.header)

    def run_inference(self, frame):
        """
        运行 YOLO 模型推理
        
        流程：预处理 -> ONNX 推理 -> 后处理
        """
        # 预处理：letterbox 缩放 + 归一化 + 转换通道顺序
        input_tensor, ratio, pad = self.preprocess(frame)
        # 运行推理，获取预测结果
        prediction = self.session.run(None, {self.input_name: input_tensor})[0]
        # 后处理：NMS + 坐标转换
        return self.postprocess(prediction, frame.shape[:2], ratio, pad)

    def preprocess(self, frame):
        """
        图像预处理
        
        步骤：
        1. Letterbox 缩放：保持宽高比，添加灰边填充到目标尺寸
        2. BGR -> RGB 颜色空间转换
        3. HWC -> CHW 格式转换（高度、宽度、通道 -> 通道、高度、宽度）
        4. 归一化：像素值 / 255.0
        5. 添加 batch 维度
        
        返回：
        - input_tensor: 预处理后的输入张量 (1, 3, H, W)
        - ratio: 缩放比例
        - pad: (pad_left, pad_top) 填充量
        """
        image, ratio, pad = self.letterbox(frame, (self.input_height, self.input_width))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.transpose(2, 0, 1)  # HWC -> CHW
        image = np.ascontiguousarray(image, dtype=np.float32) / 255.0
        image = np.expand_dims(image, axis=0)  # 添加 batch 维度
        return image, ratio, pad

    def postprocess(self, prediction, original_shape, ratio, pad):
        """
        YOLO 后处理
        
        步骤：
        1. 解析预测输出：分离边界框、类别分数
        2. 置信度过滤：去除低于阈值的检测
        3. 坐标转换：XYWH -> XYXY（中心点转角点）
        4. 坐标还原：将 letterbox 填充的坐标映射回原图
        5. NMS 非极大值抑制：去除重叠的检测框
        6. 构建检测结果列表
        
        参数：
        - prediction: ONNX 模型输出张量
        - original_shape: 原图尺寸 (height, width)
        - ratio: 缩放比例
        - pad: (pad_left, pad_top) letterbox 填充量
        """
        conf_threshold = float(self.get_parameter('confidence_threshold').value)
        iou_threshold = float(self.get_parameter('iou_threshold').value)

        # prediction 形状: (1, num_predictions, num_classes + 4)
        # 转置为 (num_predictions, num_classes + 4) 然后转置回 (num_classes + 4, num_predictions)
        pred = np.squeeze(prediction, axis=0).T
        
        # 分离边界框 (中心x, 中心y, 宽, 高) 和类别分数
        boxes_xywh = pred[:, :4]
        class_scores = pred[:, 4:]

        # 找到每个预测的最高分数类别和置信度
        class_ids = np.argmax(class_scores, axis=1)
        confidences = np.max(class_scores, axis=1)

        # 置信度过滤
        valid = confidences >= conf_threshold
        if not np.any(valid):
            return []

        boxes_xywh = boxes_xywh[valid]
        confidences = confidences[valid]
        class_ids = class_ids[valid]

        # XYWH -> XYXY 坐标转换（中心点 + 宽高 -> 左上角 + 右下角）
        boxes_xyxy = np.empty_like(boxes_xywh)
        boxes_xyxy[:, 0] = boxes_xywh[:, 0] - boxes_xywh[:, 2] * 0.5  # x1 = cx - w/2
        boxes_xyxy[:, 1] = boxes_xywh[:, 1] - boxes_xywh[:, 3] * 0.5  # y1 = cy - h/2
        boxes_xyxy[:, 2] = boxes_xywh[:, 0] + boxes_xywh[:, 2] * 0.5  # x2 = cx + w/2
        boxes_xyxy[:, 3] = boxes_xywh[:, 1] + boxes_xywh[:, 3] * 0.5  # y2 = cy + h/2

        # 还原到原图坐标（去除 letterbox 填充）
        pad_x, pad_y = pad
        boxes_xyxy[:, [0, 2]] = (boxes_xyxy[:, [0, 2]] - pad_x) / ratio
        boxes_xyxy[:, [1, 3]] = (boxes_xyxy[:, [1, 3]] - pad_y) / ratio

        # 裁剪到图像边界内
        height, width = original_shape
        boxes_xyxy[:, 0] = np.clip(boxes_xyxy[:, 0], 0, width - 1)
        boxes_xyxy[:, 2] = np.clip(boxes_xyxy[:, 2], 0, width - 1)
        boxes_xyxy[:, 1] = np.clip(boxes_xyxy[:, 1], 0, height - 1)
        boxes_xyxy[:, 3] = np.clip(boxes_xyxy[:, 3], 0, height - 1)

        # 准备 NMS 输入（需要 XYWH 格式）
        nms_boxes = []
        for box in boxes_xyxy:
            nms_boxes.append(
                [float(box[0]), float(box[1]), float(box[2] - box[0]), float(box[3] - box[1])]
            )

        # NMS 非极大值抑制
        indices = cv2.dnn.NMSBoxes(nms_boxes, confidences.tolist(), conf_threshold, iou_threshold)
        if len(indices) == 0:
            return []

        # 构建检测结果列表
        detections = []
        for idx in np.array(indices).reshape(-1):
            detections.append(
                {
                    'x1': float(boxes_xyxy[idx, 0]),
                    'y1': float(boxes_xyxy[idx, 1]),
                    'x2': float(boxes_xyxy[idx, 2]),
                    'y2': float(boxes_xyxy[idx, 3]),
                    'score': float(confidences[idx]),
                    'class_id': int(class_ids[idx]),
                    'class_name': self.class_names.get(int(class_ids[idx]), str(int(class_ids[idx]))),
                }
            )

        return detections

    @staticmethod
    def letterbox(image, new_shape, color=(114, 114, 114)):
        """
        Letterbox 缩放算法
        
        目的：保持宽高比地将图像缩放到目标尺寸，添加灰边填充
        
        步骤：
        1. 计算缩放比例（取宽高比的较小值）
        2. 按比例缩放图像
        3. 计算需要填充的像素数
        4. 均匀分配左右/上下填充
        
        示例：将 1920x1080 缩放到 640x640
        - ratio = 640/1920 = 640/1080 = 0.333
        - resized = 640x360
        - pad_w = 0, pad_h = 280 -> 上下各填充 140 像素
        
        参数：
        - image: 输入图像
        - new_shape: 目标尺寸 (height, width)
        - color: 填充颜色（灰边）
        
        返回：
        - bordered: 处理后的图像
        - ratio: 缩放比例
        - pad: (pad_left, pad_top) 填充量
        """
        height, width = image.shape[:2]
        new_height, new_width = new_shape
        
        # 计算缩放比例（取较小的比例以确保完整图像在框内）
        ratio = min(new_height / height, new_width / width)
        resized_width = int(round(width * ratio))
        resized_height = int(round(height * ratio))

        # 缩放图像
        resized = cv2.resize(image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)
        
        # 计算填充量
        pad_w = new_width - resized_width
        pad_h = new_height - resized_height
        
        # 均匀分配左右/上下填充
        pad_left = int(round(pad_w / 2.0 - 0.1))
        pad_right = int(round(pad_w / 2.0 + 0.1))
        pad_top = int(round(pad_h / 2.0 - 0.1))
        pad_bottom = int(round(pad_h / 2.0 + 0.1))

        # 添加灰边填充
        bordered = cv2.copyMakeBorder(
            resized, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=color
        )
        return bordered, ratio, (pad_left, pad_top)

    def lookup_depth(self, u, v):
        """
        查找指定像素位置的深度值
        
        使用窗口采样和中值滤波来减少深度噪声
        
        参数：
        - u, v: 像素坐标（u 是 x 列索引，v 是 y 行索引）
        
        返回：
        - 深度值（米），如果无效则返回 None
        
        深度值有效性判断：
        1. 深度值必须为正数
        2. 必须在 [min_valid_depth_m, max_valid_depth_m] 范围内
        3. 必须是有限数值（非 NaN、非 Inf）
        """
        if self.latest_depth is None:
            return None

        # 获取深度窗口参数
        half_window = max(0, int(self.get_parameter('depth_window_size').value) // 2)
        min_depth = float(self.get_parameter('min_valid_depth_m').value)
        max_depth = float(self.get_parameter('max_valid_depth_m').value)

        # 构建 ROI（感兴趣区域）
        h, w = self.latest_depth.shape[:2]
        x1 = max(0, u - half_window)
        x2 = min(w, u + half_window + 1)
        y1 = max(0, v - half_window)
        y2 = min(h, v + half_window + 1)
        roi = self.latest_depth[y1:y2, x1:x2]

        if roi.size == 0:
            return None

        # 深度图像通常为 uint16（毫米单位），转换为米
        if roi.dtype == np.uint16:
            values = roi.astype(np.float32) / 1000.0
        else:
            values = roi.astype(np.float32)

        # 过滤无效深度值
        values = values[np.isfinite(values)]  # 去除 NaN 和 Inf
        values = values[(values > 0.0) & (values >= min_depth) & (values <= max_depth)]
        if values.size == 0:
            return None

        # 返回中值深度（抗噪声）
        return float(np.median(values))

    def is_depth_fresh(self, color_header):
        """
        检查深度图像是否足够新鲜（延迟在可接受范围内）
        
        设计原因：
        深度图像和彩色图像是异步接收的，如果深度图像太旧，
        会导致 3D 坐标计算不准确（目标已经移动）
        
        参数：
        - color_header: 当前彩色图像的头信息（包含时间戳）
        
        返回：
        - True 如果深度图像足够新鲜
        - False 如果深度图像过期或不可用
        """
        if self.latest_depth_header is None:
            return False

        max_age_sec = float(self.get_parameter('max_depth_age_sec').value)
        if max_age_sec <= 0.0:
            return True

        # 提取时间戳
        color_stamp = Time.from_msg(color_header.stamp)
        depth_stamp = Time.from_msg(self.latest_depth_header.stamp)
        
        # 处理时间戳为 0 的特殊情况（可能发生在模拟器中）
        if color_stamp.nanoseconds == 0 or depth_stamp.nanoseconds == 0:
            return True

        # 计算深度图像和彩色图像的时间差
        age_sec = abs((color_stamp - depth_stamp).nanoseconds) / 1e9
        return age_sec <= max_age_sec

    def project_pixel_to_3d(self, u, v, depth_m):
        """
        将像素坐标和深度值投影到 3D 世界坐标系
        
        相机投影模型（针孔相机模型）：
        
              Z (深度)
             /  
            /   
           /    * 目标点 (X, Y, Z)
          /   /|
         /  / |
        O  /  | Y
        / /   |
        //----|-------> X
        /     |
      u       |
       \\
        v
        
        投影公式（反向投影）：
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        Z = depth_m
        
        参数：
        - u, v: 像素坐标
        - depth_m: 深度值（米）
        
        返回：
        - (x, y, z) 3D 坐标（米）
        """
        # 从相机内参矩阵提取参数
        fx = self.intrinsics[0]   # 焦距 x
        fy = self.intrinsics[4]   # 焦距 y
        cx = self.intrinsics[2]   # 主点 x
        cy = self.intrinsics[5]   # 主点 y

        # 应用投影公式
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return x, y, z

    @staticmethod
    def extract_camera_position(transform):
        if transform is None:
            return None
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    @staticmethod
    def format_xyz(values):
        if values is None:
            return 'N/A'
        return f'({values[0]:.3f},{values[1]:.3f},{values[2]:.3f})'

    def confirm_point_callback(self, _msg):
        self.enter_target_publish_hold('confirmed point published')

    def execution_busy_callback(self, msg):
        busy = bool(msg.data)
        was_busy = self.last_execution_busy
        self.last_execution_busy = busy

        if busy:
            self.execution_busy_seen_true = True
            return

        if self.target_publish_hold and (was_busy or self.execution_busy_seen_true):
            self.release_target_publish_hold('grab execution completed')

    def enter_target_publish_hold(self, reason):
        if self.target_publish_hold:
            return
        self.target_publish_hold = True
        self.target_publish_hold_start_ns = self.get_clock().now().nanoseconds
        self.execution_busy_seen_true = self.last_execution_busy
        self.get_logger().info(f'Pause /pepper/target_point publish: {reason}')

    def release_target_publish_hold(self, reason):
        if not self.target_publish_hold:
            return
        self.target_publish_hold = False
        self.target_publish_hold_start_ns = 0
        self.execution_busy_seen_true = False
        self.get_logger().info(f'Resume /pepper/target_point publish: {reason}')

    def check_target_publish_hold_timeout(self):
        if not self.target_publish_hold:
            return

        timeout_sec = float(self.get_parameter('target_publish_hold_timeout_sec').value)
        if timeout_sec <= 0.0:
            return

        elapsed_sec = (
            self.get_clock().now().nanoseconds - self.target_publish_hold_start_ns
        ) / 1e9
        if elapsed_sec < timeout_sec:
            return

        self.get_logger().warn(
            f'/pepper/target_point publish hold timeout ({elapsed_sec:.1f}s >= {timeout_sec:.1f}s).'
        )
        self.release_target_publish_hold('hold timeout')

    def should_log_position_status(self):
        interval_sec = max(0.0, float(self.get_parameter('position_log_interval_sec').value))
        now_ns = self.get_clock().now().nanoseconds
        if interval_sec <= 0.0:
            self.last_position_log_time_ns = now_ns
            return True
        if self.last_position_log_time_ns == 0:
            self.last_position_log_time_ns = now_ns
            return True
        elapsed_sec = (now_ns - self.last_position_log_time_ns) / 1e9
        if elapsed_sec >= interval_sec:
            self.last_position_log_time_ns = now_ns
            return True
        return False

    def log_position_status(self, cam_in_base, bing_in_camera, bing_in_base):
        if not self.should_log_position_status():
            return
        target_frame = str(self.get_parameter('overlay_target_frame').value).strip()
        cam_text = self.format_xyz(cam_in_base) if cam_in_base is not None else 'TF_FAIL'
        bing_cam_text = self.format_xyz(bing_in_camera) if bing_in_camera is not None else 'N/A'
        if bing_in_camera is None:
            bing_base_text = 'N/A'
        elif bing_in_base is None:
            bing_base_text = 'TF_FAIL'
        else:
            bing_base_text = self.format_xyz(bing_in_base)
        self.get_logger().info(
            f'Position status [{target_frame}]: '
            f'cam_in_base={cam_text}, '
            f'bing_in_camera={bing_cam_text}, '
            f'bing_in_base={bing_base_text}, '
            f'target_publish_hold={"ON" if self.target_publish_hold else "OFF"}'
        )

    def draw_position_overlay(self, frame, cam_in_base, bing_in_camera, bing_in_base):
        if not bool(self.get_parameter('show_position_overlay').value):
            return
        target_frame = str(self.get_parameter('overlay_target_frame').value).strip()
        lines = []
        colors = []

        if cam_in_base is None:
            lines.append(f'cam_in_{target_frame}: TF_FAIL')
            colors.append((0, 0, 255))
        else:
            lines.append(f'cam_in_{target_frame}: {self.format_xyz(cam_in_base)} m')
            colors.append((0, 220, 0))

        if bing_in_camera is None:
            lines.append('bing_in_camera: N/A')
            colors.append((0, 200, 255))
            lines.append(f'bing_in_{target_frame}: N/A')
            colors.append((0, 200, 255))
        else:
            lines.append(f'bing_in_camera: {self.format_xyz(bing_in_camera)} m')
            colors.append((0, 220, 0))
            if bing_in_base is None:
                lines.append(f'bing_in_{target_frame}: TF_FAIL')
                colors.append((0, 0, 255))
            else:
                lines.append(f'bing_in_{target_frame}: {self.format_xyz(bing_in_base)} m')
                colors.append((0, 220, 0))

        origin_x = 18
        origin_y = 24
        for index, line in enumerate(lines):
            y = origin_y + index * 22
            cv2.putText(
                frame,
                line,
                (origin_x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                colors[index],
                2,
            )

    def lookup_overlay_transform(self, source_header):
        """
        查找 source_header.frame_id 到 overlay_target_frame 的 TF 变换
        """
        target_frame = str(self.get_parameter('overlay_target_frame').value).strip()
        timeout_sec = float(self.get_parameter('overlay_tf_timeout_sec').value)
        if not target_frame or not source_header.frame_id:
            return None

        source_stamp = Time.from_msg(source_header.stamp)
        lookup_time = source_stamp if source_stamp.nanoseconds != 0 else Time()
        timeout = Duration(seconds=max(0.0, timeout_sec))

        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_header.frame_id,
                lookup_time,
                timeout=timeout,
            )
        except TransformException as stamp_exc:
            # 当图像时间戳明显超前于当前 TF buffer 时，回退到 latest TF，
            # 避免在 CPU 推理负载高时持续出现 extrapolation into the future。
            if lookup_time.nanoseconds != 0:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        source_header.frame_id,
                        Time(),
                        timeout=timeout,
                    )
                    self.log_overlay_tf_fallback(
                        source_header.frame_id, target_frame, stamp_exc
                    )
                    return transform
                except TransformException:
                    pass
            self.warn_overlay_tf_unavailable(source_header.frame_id, target_frame, stamp_exc)
            return None

    def log_overlay_tf_fallback(self, source_frame, target_frame, stamp_exc):
        interval_sec = max(0.0, float(self.get_parameter('overlay_tf_warning_interval_sec').value))
        now_ns = self.get_clock().now().nanoseconds
        if interval_sec > 0.0:
            elapsed_sec = (now_ns - self.last_overlay_tf_fallback_log_time_ns) / 1e9
            if self.last_overlay_tf_fallback_log_time_ns != 0 and elapsed_sec < interval_sec:
                return
        self.last_overlay_tf_fallback_log_time_ns = now_ns
        self.get_logger().warn(
            'Overlay TF lookup with image timestamp failed, fallback to latest TF. '
            f'{source_frame} -> {target_frame}. cause={stamp_exc}'
        )

    def warn_overlay_tf_unavailable(self, source_frame, target_frame, exc):
        interval_sec = max(0.0, float(self.get_parameter('overlay_tf_warning_interval_sec').value))
        now_ns = self.get_clock().now().nanoseconds
        if interval_sec > 0.0:
            elapsed_sec = (now_ns - self.last_overlay_tf_warning_time_ns) / 1e9
            if self.last_overlay_tf_warning_time_ns != 0 and elapsed_sec < interval_sec:
                return
        self.last_overlay_tf_warning_time_ns = now_ns
        self.get_logger().warn(
            f'Cannot transform for overlay {source_frame} -> {target_frame}: {exc}'
        )

    @staticmethod
    def transform_point_to_overlay_frame(x, y, z, source_header, transform):
        """
        将点从 source_header.frame_id 变换到 overlay_target_frame
        """
        point = PointStamped()
        point.header = source_header
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)
        transformed = do_transform_point(point, transform)
        return transformed.point.x, transformed.point.y, transformed.point.z

    def publish_target_point(self, x, y, z, header):
        """
        发布目标点的 3D 坐标
        
        用于机械臂抓取控制，发布的是作物茎部的世界坐标
        
        参数：
        - x, y, z: 3D 坐标（米）
        - header: ROS 消息头（包含时间戳和坐标系）
        """
        point = PointStamped()
        point.header = header
        point.point.x = float(x)
        point.point.y = float(y)
        point.point.z = float(z)
        self.target_point_pub.publish(point)

    def publish_all_points(self, targets, header):
        """
        发布所有有效检测点为 PoseArray（位置有效，姿态为单位四元数）

        参数：
        - targets: [{'xyz': (x, y, z), ...}, ...]
        - header: 原始图像头信息（坐标系通常为相机光学坐标系）
        """
        msg = PoseArray()
        msg.header = header

        for item in targets:
            x, y, z = item['xyz']
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.all_points_pub.publish(msg)

    def draw_detection(self, frame, detection, u, v, label_main, label_detail, highlight):
        """
        在图像上绘制检测结果标注
        
        标注内容：
        - 边界框（矩形）
        - 中心点（圆点）
        - 标签文字（类别名 + 置信度 + 3D 坐标）
        
        颜色语义：
        - 绿色 (0, 220, 0)：有效的 3D 坐标
        - 黄色 (0, 255, 255)：中心点
        - 红色 (0, 0, 255)：无效的 3D 坐标
        - 橙色 (0, 165, 255)：无效的中心点
        """
        x1 = int(round(detection['x1']))
        y1 = int(round(detection['y1']))
        x2 = int(round(detection['x2']))
        y2 = int(round(detection['y2']))
        
        if highlight:
            box_color = (0, 220, 0)      # 绿色 - 有效
            point_color = (0, 255, 255)  # 黄色 - 中心点
        else:
            box_color = (0, 0, 255)      # 红色 - 无效
            point_color = (0, 165, 255)  # 橙色 - 中心点

        # 绘制边界框
        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
        # 绘制中心点
        cv2.circle(frame, (u, v), 4, point_color, -1)
        # 两行标签：第一行类别+置信度，第二行 base_link 关系
        line1_y = max(20, y1 - 12)
        line2_y = min(frame.shape[0] - 10, line1_y + 20)
        cv2.putText(
            frame,
            label_main,
            (x1, line1_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            box_color,
            2,
        )
        cv2.putText(
            frame,
            label_detail,
            (x1, line2_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            box_color,
            2,
        )

    def publish_debug(self, frame, header):
        """
        发布带标注的调试图像
        
        用于可视化检测结果，包括：
        - 检测框
        - 中心点
        - 类别名称和置信度
        - 3D 坐标信息
        """
        if not self.get_parameter('publish_debug_image').value:
            return
        debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        debug_msg.header = header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    """
    节点入口函数
    
    执行流程：
    1. 初始化 ROS2 客户端库
    2. 创建节点实例
    3. 使用 rclpy.spin 保持节点运行，响应回调
    4. 处理 Ctrl+C 退出
    5. 销毁节点并关闭 ROS2
    """
    rclpy.init(args=args)
    try:
        node = pepperDetector3D()
    except Exception as exc:
        print(f'[pepper_detector] startup failed: {exc}')
        rclpy.shutdown()
        raise

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
