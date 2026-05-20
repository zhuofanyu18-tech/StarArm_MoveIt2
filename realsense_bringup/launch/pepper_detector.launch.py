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

    model_arg = DeclareLaunchArgument(
        'model_path',
        default_value=os.path.join(package_dir, 'config', 'best.onnx'),
        description='path to the ONNX model file.',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch standalone RViz (normally keep false and use moveit.rviz).',
    )
    start_realsense_arg = DeclareLaunchArgument(
        'start_realsense',
        default_value='true',
        description='Launch realsense2_camera_node from this launch file.',
    )
    calib_yaml_arg = DeclareLaunchArgument(
        'calib_yaml_path',
        default_value=os.path.join(package_dir, 'config', 'd435i_color.yaml'),
        description='Path to calibrated color camera intrinsics yaml.',
    )
    venv_site_packages_arg = DeclareLaunchArgument(
        'venv_site_packages',
        default_value='',
        description='Site-packages path of the dedicated perception virtualenv.',
    )
    auto_grab_arg = DeclareLaunchArgument(
        'auto_grab',
        default_value='true',
        description='Automatically select nearest reachable pepper and publish /grab_pose.',
    )
    enable_confirm_pipeline_arg = DeclareLaunchArgument(
        'enable_confirm_pipeline',
        default_value='true',
        description='Enable bing stable-confirm pipeline and confirmed grab publisher.',
    )
    enable_legacy_selector_arg = DeclareLaunchArgument(
        'enable_legacy_selector',
        default_value='false',
        description='Enable legacy pepper_grasp_selector pipeline.',
    )
    overlay_target_frame_arg = DeclareLaunchArgument(
        'overlay_target_frame',
        default_value='base_link',
        description='Target frame used by detector image overlay text.',
    )
    overlay_tf_timeout_sec_arg = DeclareLaunchArgument(
        'overlay_tf_timeout_sec',
        default_value='0.2',
        description='TF lookup timeout (seconds) for detector overlay transform.',
    )
    color_profile_arg = DeclareLaunchArgument(
        'color_profile',
        default_value='640x480x15',
        description='RealSense color stream profile (widthxheightxfps).',
    )
    depth_profile_arg = DeclareLaunchArgument(
        'depth_profile',
        default_value='640x480x15',
        description='RealSense depth stream profile (widthxheightxfps).',
    )
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        parameters=[
            {
                'camera_namespace': 'camera',
                'camera_name': 'camera',
                'align_depth.enable': True,
                'base_frame_id': 'camera_link',
                'publish_tf': False,
                'rgb_camera.profile': LaunchConfiguration('color_profile'),
                'depth_module.profile': LaunchConfiguration('depth_profile'),
            }
        ],
        condition=IfCondition(LaunchConfiguration('start_realsense')),
    )
    detector_node = Node(
        package='realsense_bringup',
        executable='pepper_detector',
        name='pepper_detector',
        output='screen',
        parameters=[
            {
                'model_path': LaunchConfiguration('model_path'),
                'camera_info_yaml': LaunchConfiguration('calib_yaml_path'),
                'overlay_target_frame': LaunchConfiguration('overlay_target_frame'),
                'overlay_tf_timeout_sec': ParameterValue(
                    LaunchConfiguration('overlay_tf_timeout_sec'), value_type=float
                ),
            }
        ],
        additional_env={
            'PERCEPTION_SITE_PACKAGES': LaunchConfiguration('venv_site_packages'),
        },
    )
    transformer_node = Node(
        package='realsense_bringup',
        executable='pepper_point_transformer',
        name='pepper_point_transformer',
        output='screen',
    )
    selector_node = Node(
        package='realsense_bringup',
        executable='pepper_grasp_selector',
        name='pepper_grasp_selector',
        output='screen',
        parameters=[
            {
                'enable_auto_grab': ParameterValue(
                    LaunchConfiguration('auto_grab'), value_type=bool
                ),
                'candidate_marker_size_m': 0.04,
                'selected_marker_size_m': 0.045,
            }
        ],
        condition=IfCondition(LaunchConfiguration('enable_legacy_selector')),
    )
    confirmer_node = Node(
        package='realsense_bringup',
        executable='pepper_bing_confirmer',
        name='pepper_bing_confirmer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_confirm_pipeline')),
    )
    confirmed_transformer_node = Node(
        package='realsense_bringup',
        executable='pepper_confirmed_transformer',
        name='pepper_confirmed_transformer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_confirm_pipeline')),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(package_dir, 'rviz', 'pepper_detector.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        model_arg,
        rviz_arg,
        start_realsense_arg,
        calib_yaml_arg,
        venv_site_packages_arg,
        auto_grab_arg,
        enable_confirm_pipeline_arg,
        enable_legacy_selector_arg,
        overlay_target_frame_arg,
        overlay_tf_timeout_sec_arg,
        color_profile_arg,
        depth_profile_arg,
        realsense_node,
        detector_node,
        transformer_node,
        selector_node,
        confirmer_node,
        confirmed_transformer_node,
        rviz_node,
    ])
