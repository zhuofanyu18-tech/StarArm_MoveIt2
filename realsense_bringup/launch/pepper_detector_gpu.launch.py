import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_dir = get_package_share_directory('realsense_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=os.path.join(package_dir, 'config', 'best.onnx')),
        DeclareLaunchArgument('calib_yaml_path', default_value=os.path.join(package_dir, 'config', 'd435i_color.yaml')),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('color_profile', default_value='1280x720x30'),
        DeclareLaunchArgument('depth_profile', default_value='1280x720x30'),
        DeclareLaunchArgument('venv_site_packages', default_value=''),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            output='screen',
            parameters=[{
                'camera_namespace': 'camera',
                'camera_name': 'camera',
                'align_depth.enable': True,
                'base_frame_id': 'camera_link',
                'publish_tf': False,
                'rgb_camera.profile': LaunchConfiguration('color_profile'),
                'depth_module.profile': LaunchConfiguration('depth_profile'),
            }],
            condition=IfCondition(LaunchConfiguration('start_realsense')),
        ),

        Node(
            package='realsense_bringup',
            executable='pepper_detector_gpu',
            name='pepper_detector_gpu',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'camera_info_yaml': LaunchConfiguration('calib_yaml_path'),
            }],
            additional_env={
                'PERCEPTION_SITE_PACKAGES': LaunchConfiguration('venv_site_packages'),
            },
        ),
    ])
