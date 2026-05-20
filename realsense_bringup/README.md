## Camera bringup

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

Or start RealSense directly inside detector launch (default behavior):

```bash
ros2 launch realsense_bringup pepper_detector.launch.py
```

## Perception environment

Use a dedicated virtualenv so the ONNX runtime does not pollute the ROS Python environment:

```bash
cd /home/yu/starbot_arm_ws
python3 -m venv .venv_perception --system-site-packages
source .venv_perception/bin/activate
python -m pip install --upgrade pip
python -m pip install -r src/realsense_bringup/requirements-perception.txt
```

The `numpy<2` pin is required because ROS Humble `cv2` and `cv_bridge` are built against NumPy 1.x.

## Build the ROS workspace

```bash
cd /home/yu/starbot_arm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select realsense_bringup
```

## Run the pepper detector

Open a new terminal:

```bash
cd /home/yu/starbot_arm_ws
source .venv_perception/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense_bringup pepper_detector.launch.py
```

Use a custom color calibration YAML (for example `d435i_color.yaml`):

```bash
ros2 launch realsense_bringup pepper_detector.launch.py \
  calib_yaml_path:=/home/yu/starbot_arm_ws/src/realsense_bringup/config/d435i_color.yaml
```

Use `rviz:=false` (default) to avoid opening a second RViz window when MoveIt RViz is already running:

```bash
ros2 launch realsense_bringup pepper_detector.launch.py rviz:=false
```

Default RealSense profiles are set to lower CPU load:

- `color_profile:=640x480x15`
- `depth_profile:=640x480x15`

You can tune them at launch time, for example:

```bash
ros2 launch realsense_bringup pepper_detector.launch.py \
  color_profile:=848x480x30 \
  depth_profile:=848x480x30
```

Default pipeline (`enable_confirm_pipeline:=true`) subscribes to:

- `/camera/camera/color/image_raw`
- `/camera/camera/aligned_depth_to_color/image_raw`
- `/camera/camera/color/camera_info`
- `/grab/execution_busy` (from `star_arm_grab`)
- `/pepper/confirmed_point_camera` (for detector publish-hold trigger)

Default pipeline publishes:

- `/detect/image`
- `/pepper/target_point`
- `/pepper/detected_points` (all valid 3D detections in camera frame)
- `/pepper/confirmed_point_camera` (stable confirmed `bing` in camera frame)
- `/pepper/confirmed_point_base` (confirmed point transformed to `base_link`)
- `/grab_pose` (published once after stable confirm, for `star_arm_grab`)
- TF `base_link -> bing`

Behavior:

- `rou` and `bing` are both drawn on `/detect/image`
- each bounding box shows class name, confidence, and 3D coordinates when depth is valid
- `/detect/image` also overlays `cam_in_base`, `bing_in_camera`, `bing_in_base`
- terminal logs the same position status with `position_log_interval_sec` (default 1.0s)
- `rou` means fruit, `bing` means stem
- all valid peppers are published to `/pepper/detected_points`
- `pepper_bing_confirmer` confirms target after `confirm_frames=5` and `stability_radius_m=0.015`
- after confirming, `pepper_bing_confirmer` waits for `/grab/execution_busy` to go `true -> false`
- detector pauses `/pepper/target_point` publishing during execution (detection/overlay stay active)
- publish hold can auto-release by timeout (`wait_completion_timeout_sec`, `target_publish_hold_timeout_sec`)
- `pepper_confirmed_transformer` converts confirmed point to `base_link` and publishes `/grab_pose`
- `/pepper/target_point` and `/pepper/target_point_base` remain available for compatibility

## Legacy selector fallback

Enable old selector pipeline only:

```bash
ros2 launch realsense_bringup pepper_detector.launch.py \
  enable_confirm_pipeline:=false \
  enable_legacy_selector:=true \
  auto_grab:=true
```

Legacy selector topics (when enabled):

- `/pepper/detected_markers`
- `/pepper/selected_point_base`
- `/grab_pose`

## TF diagnostics

Make sure robot TF is running (`real.launch.py` / `robot_state_publisher`) and verify:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo base_link bing
```
