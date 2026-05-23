import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('realsense_bringup')

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(pkg_share, 'config', 'best.onnx'),
        description='ONNX model path.',
    )
    calib_yaml_arg = DeclareLaunchArgument(
        'camera_info_yaml',
        default_value='',
        description='Path to camera calibration yaml (optional, falls back to camera_info topic).',
    )
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU (CUDA) detector instead of CPU.',
    )
    start_realsense_arg = DeclareLaunchArgument(
        'start_realsense',
        default_value='true',
        description='Launch realsense2_camera_node.',
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.35',
        description='YOLO confidence threshold.',
    )
    debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='Publish annotated debug image.',
    )

    # RealSense 相机节点
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']
        ),
        condition=IfCondition(LaunchConfiguration('start_realsense')),
        launch_arguments={
            'camera_name': 'camera',
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'pointcloud.enable': 'true',
        }.items(),
    )

    # GPU 检测器
    gpu_node = Node(
        package='realsense_bringup',
        executable='pepper_detector_gpu',
        name='pepper_detector_gpu',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gpu')),
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'camera_info_yaml': LaunchConfiguration('camera_info_yaml'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }],
    )

    # CPU 检测器
    cpu_node = Node(
        package='realsense_bringup',
        executable='pepper_detector',
        name='pepper_detector_cpu',
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_gpu')),
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'camera_info_yaml': LaunchConfiguration('camera_info_yaml'),
            'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
        }],
    )

    return LaunchDescription([
        model_path_arg,
        calib_yaml_arg,
        use_gpu_arg,
        start_realsense_arg,
        confidence_arg,
        debug_image_arg,
        realsense_node,
        gpu_node,
        cpu_node,
    ])
