import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CircleTracker(Node):
    def __init__(self):
        super().__init__('circle_tracker')
        self.bridge = CvBridge()
        
        # --- 动态参数调整区 ---
        # 针对近距离物体，通常需要更大的半径范围和更宽松的检测阈值
        self.declare_parameter('min_radius', 50)  # 近距离圆很大，调大最小半径
        self.declare_parameter('max_radius', 400) # 调大最大半径
        self.declare_parameter('hough_param1', 30) # Canny边缘检测高阈值，调低有助于检测平滑边缘
        self.declare_parameter('hough_param2', 20) # 圆心累加器阈值，调低越容易检测到圆（也容易误检）
        self.declare_parameter('show_mask', True) 

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        
        self.latest_depth_img = None
        self.intrinsics = None
        self.hsv_kernel = np.ones((5, 5), np.uint8)

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f"Parameter updated: {param.name} = {param.value}")
        return SetParametersResult(successful=True)

    def info_callback(self, msg):
        self.intrinsics = msg.k

    def depth_callback(self, msg):
        self.latest_depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def image_callback(self, msg):
        if self.latest_depth_img is None or self.intrinsics is None:
            return

        # 读取当前参数
        min_r = self.get_parameter('min_radius').value
        max_r = self.get_parameter('max_radius').value
        p1 = self.get_parameter('hough_param1').value
        p2 = self.get_parameter('hough_param2').value
        show_mask = self.get_parameter('show_mask').value

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 红色掩膜处理 (保持不变，既然你说mask很清晰)
        mask1 = cv2.inRange(hsv, np.array([0, 130, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 130, 70]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.hsv_kernel)
        mask = cv2.GaussianBlur(mask, (9, 9), 2)

        if show_mask: cv2.imshow("Red Mask", mask)

        # 霍夫圆检测
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.2, minDist=min_r*2,
                                   param1=p1, param2=p2, minRadius=min_r, maxRadius=max_r)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            # 找到最大的圆 (通常是最靠近镜头的目标)
            largest_circle = max(circles[0, :], key=lambda c: c[2])
            u, v, r = largest_circle[0], largest_circle[1], largest_circle[2]
            
            # 越界检查
            if v < self.latest_depth_img.shape[0] and u < self.latest_depth_img.shape[1]:
                depth_value = self.latest_depth_img[v, u]
                
                # --- 核心修改区：深度无效时的处理 ---
                if depth_value == 0 or depth_value < 170: # 小于17cm通常数据不可靠
                    # 【情况1】视觉找到了圆，但深度无效 (显示红色警告)
                    cv2.circle(frame, (u, v), r, (0, 0, 255), 3) # 红圈
                    cv2.circle(frame, (u, v), 3, (0, 0, 255), -1) # 红点
                    cv2.putText(frame, "Too Close / No Depth!", (u-60, v-r-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                else:
                    # 【情况2】视觉和深度都正常 (显示绿色结果)
                    z = float(depth_value) / 1000.0
                    x = (u - self.intrinsics[2]) * z / self.intrinsics[0]
                    y = (v - self.intrinsics[5]) * z / self.intrinsics[4]
                    cv2.circle(frame, (u, v), r, (0, 255, 0), 2) # 绿圈
                    cv2.circle(frame, (u, v), 3, (0, 255, 0), -1) # 绿点
                    cv2.putText(frame, f"X:{x:.2f} Y:{y:.2f} Z:{z:.2f}m", (u-60, v-r-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("Detection Result", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CircleTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()