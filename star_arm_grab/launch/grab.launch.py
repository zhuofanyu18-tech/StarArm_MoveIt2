import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    description_dir = get_package_share_directory("star_arm")
    default_model_path = os.path.join(description_dir, "urdf", "star_arm.urdf.xacro")

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to the robot URDF or Xacro file.",
    )
    grab_pose_z_offset_arg = DeclareLaunchArgument(
        "grab_pose_z_offset_m",
        default_value="0.07",
        description="Extra z offset applied to the averaged grab target (m).",
    )
    grab_pose_x_offset_arg = DeclareLaunchArgument(
        "grab_pose_x_offset_m",
        default_value="0.02",
        description="Extra x offset applied to the averaged grab target (m).",
    )
    stability_samples_arg = DeclareLaunchArgument(
        "stability_samples",
        default_value="4",
        description="Number of stable rou detections required before grab.",
    )
    stability_threshold_arg = DeclareLaunchArgument(
        "stability_threshold_m",
        default_value="0.015",
        description="Max allowed distance (m) of detections from centroid for stability.",
    )
    camera_frame_arg = DeclareLaunchArgument(
        "camera_frame",
        default_value="camera_color_optical_frame",
        description="Camera optical frame for TF transform to base_link.",
    )
    place_named_target_arg = DeclareLaunchArgument(
        "place_named_target",
        default_value="drop_zone",
        description="Arm named target used for pepper placement.",
    )
    detect_named_target_arg = DeclareLaunchArgument(
        "detect_named_target",
        default_value="detect",
        description="Arm named target used to return to detection pose.",
    )
    gripper_close_named_target_arg = DeclareLaunchArgument(
        "gripper_close_named_target",
        default_value="gripper_close",
        description="Gripper named target used to close the gripper.",
    )
    gripper_open_named_target_arg = DeclareLaunchArgument(
        "gripper_open_named_target",
        default_value="gripper_open",
        description="Gripper named target used to open the gripper.",
    )
    gripper_settle_time_ms_arg = DeclareLaunchArgument(
        "gripper_settle_time_ms",
        default_value="2000",
        description="Delay (ms) after gripper close/open to ensure servo physically settles.",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="star_arm_movit_config")
        .robot_description(file_path=default_model_path)
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": robot_description}

    grab_node = Node(
        package="star_arm_grab",
        executable="grab_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "lock_joint5": False,
                "locked_joint_name": "joint5",
                "locked_joint_position_rad": 0.0,
                "lock_joint_tolerance_rad": 0.03,
                "grab_pose_z_offset_m": ParameterValue(
                    LaunchConfiguration("grab_pose_z_offset_m"), value_type=float
                ),
                "grab_pose_x_offset_m": ParameterValue(
                    LaunchConfiguration("grab_pose_x_offset_m"), value_type=float
                ),
                "stability_samples": ParameterValue(
                    LaunchConfiguration("stability_samples"), value_type=int
                ),
                "stability_threshold_m": ParameterValue(
                    LaunchConfiguration("stability_threshold_m"), value_type=float
                ),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "place_named_target": LaunchConfiguration("place_named_target"),
                "detect_named_target": LaunchConfiguration("detect_named_target"),
                "gripper_close_named_target": LaunchConfiguration(
                    "gripper_close_named_target"
                ),
                "gripper_open_named_target": LaunchConfiguration(
                    "gripper_open_named_target"
                ),
                "gripper_settle_time_ms": ParameterValue(
                    LaunchConfiguration("gripper_settle_time_ms"), value_type=int
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            grab_pose_z_offset_arg,
            grab_pose_x_offset_arg,
            stability_samples_arg,
            stability_threshold_arg,
            camera_frame_arg,
            place_named_target_arg,
            detect_named_target_arg,
            gripper_close_named_target_arg,
            gripper_open_named_target_arg,
            gripper_settle_time_ms_arg,
            grab_node,
        ]
    )
