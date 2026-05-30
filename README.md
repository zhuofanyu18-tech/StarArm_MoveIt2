## 机械臂的控制
    ---- ESP32 控制 ----
    <!-- ls /dev/tty* sudo chmod 777 /dev/ttyUSB0
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
    ros2 launch star_arm_movit_config real.launch.py
    ros2 launch star_arm_grab grab.launch.py -->
# 第一步
    ---- 目前的基于ros2_control的控制 ----
    ros2 launch star_arm_hardware moveit_real_hardware.launch.py serial_port:=/dev/ttyACM0

## 草莓的识别
    source .venv_perception/bin/activate

    source /opt/ros/humble/setup.bash
    目前可以不启动
    (ros2 launch realsense_bringup pepper_detector.launch.py \
    calib_yaml_path:=/home/yu/starbot_arm_ws/src/realsense_bringup/config/d435i_color.yaml)
    这个里面自带启动realsense D435i
    
    ros2 launch realsense_bringup pepper_detector.launch.py rviz:=false

    source /home/yu/starbot_arm_ws/.venv_perception/bin/activate

# 第二步
    ---- 启动相机 + YOLO 检测 ----
    source .venv_perception/bin/activate   # 激活感知虚拟环境
    ros2 launch realsense_bringup pepper_rou_detector.launch.py


# 第三步
    ---- 启动抓取节点 ----
    ros2 launch star_arm_grab grab.launch.py



## 手动模拟抓取目标（跳过检测器测试抓取）
ros2 topic pub /pepper/rou_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: camera_color_optical_frame}, pose: {position: {x: 0.0, y: 0.0, z: 0.40}, orientation: {x: 0, y: 0, z: 0, w: 1}}}" -r 5



# 杀掉所有包含 "trajectory" 或 "bridge" 字眼的进程
pkill -9 python3
pkill -9 -f trajectory


## 对于 foxglove 的配置和启动
ros2 launch rosbridge_server rosbridge_websocket_launch.xml


## 本地大语言模型的启动

# 终端 1：启动 llama-server
cd ~/llama.cpp
./build/bin/llama-server -m models/qwen2.5-7b-q4_k_m.gguf --host 0.0.0.0 --port 8080 -c 4096

# 终端 2：启动桥接节点
source /opt/ros/humble/setup.bash
source ~/starbot_arm_ws/install/setup.bash
ros2 run llm_bridge llm_bridge_node

# 终端 3：发一次消息
source /opt/ros/humble/setup.bash
ros2 topic pub --once /llm/query std_msgs/msg/String "data: '你好，说说机械臂有几个关节？'"
ros2 topic echo /llm/response


ros2 topic pub /stepper_motor_target std_msgs/msg/Float32MultiArray "{data: [10.0, 10.0]}" --once