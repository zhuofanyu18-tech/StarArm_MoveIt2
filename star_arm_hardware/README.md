# 第一步：只测试硬件层，确认 USB 舵机能通信
ros2 launch star_arm_hardware real_hardware.launch.py serial_port:=/dev/ttyACM0 use_rviz:=False

# 第二步：测试通过后，用 MoveIt2 控制
ros2 launch star_arm_hardware moveit_real_hardware.launch.py serial_port:=/dev/ttyACM0

# 关节限制验证（确保转换公式不超出舵机范围）
# joint1: -1.60 → 伺服值 = -1.60*652+2048 = 1005 ✓
# joint1: +1.60 → 伺服值 = +1.60*652+2048 = 3091 ✓
# joint3: +3.00 → 伺服值 = +3.00*652+2048 = 4004 ✓ (0~4095 内)

