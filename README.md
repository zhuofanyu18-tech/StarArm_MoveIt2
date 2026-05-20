# 机械臂的控制
    <!-- ls /dev/tty*
    sudo chmod 777 /dev/ttyUSB0
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
    ros2 launch star_arm_movit_config real.launch.py
    ros2 launch star_arm_grab grab.launch.py -->
    ros2 launch star_arm_hardware moveit_real_hardware.launch.py serial_port:=/dev/ttyACM0

# 草莓的识别
    source .venv_perception/bin/activate

    source /opt/ros/humble/setup.bash
    目前可以不启动
    (ros2 launch realsense_bringup pepper_detector.launch.py \
    calib_yaml_path:=/home/yu/starbot_arm_ws/src/realsense_bringup/config/d435i_color.yaml)
    这个里面自带启动realsense D435i
    ros2 launch realsense_bringup pepper_detector.launch.py rviz:=false


    source /home/yu/starbot_arm_ws/.venv_perception/bin/activate

ros2 topic pub /grab_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: base_link}, pose: {position: {x: 0.30, y: 0.00, z: 0.30}, orientation: {x: 0, y: 0, z: 0, w: 1}}}" -1


# 杀掉所有包含 "trajectory" 或 "bridge" 字眼的进程
pkill -9 python3
pkill -9 -f trajectory