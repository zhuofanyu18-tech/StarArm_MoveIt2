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
        default_value="0.03",
        description="Extra z offset applied to every incoming /grab_pose target.",
    )
    place_named_target_arg = DeclareLaunchArgument(
        "place_named_target",
        default_value="fangzhi",
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
        remappings=[("/joint_states", "/joint_states_stamped")],
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
                "place_named_target": LaunchConfiguration("place_named_target"),
                "detect_named_target": LaunchConfiguration("detect_named_target"),
                "gripper_close_named_target": LaunchConfiguration(
                    "gripper_close_named_target"
                ),
                "gripper_open_named_target": LaunchConfiguration(
                    "gripper_open_named_target"
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            grab_pose_z_offset_arg,
            place_named_target_arg,
            detect_named_target_arg,
            gripper_close_named_target_arg,
            gripper_open_named_target_arg,
            grab_node,
        ]
    )
