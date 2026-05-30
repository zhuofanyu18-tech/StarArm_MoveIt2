import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def load_joint_config():
    description_dir = get_package_share_directory("star_arm")
    joint_config_path = os.path.join(
        description_dir,
        "config",
        "joint_names_star_arm.yaml",
    )
    with open(joint_config_path, "r", encoding="utf-8") as stream:
        joint_config = yaml.safe_load(stream) or {}

    arm_joint_names = joint_config.get("arm_joint_names", [])
    gripper_joint_names = joint_config.get("gripper_joint_names", [])

    if not arm_joint_names:
        raise RuntimeError(
            "arm_joint_names is empty in joint_names_star_arm.yaml"
        )
    if not gripper_joint_names:
        raise RuntimeError(
            "gripper_joint_names is empty in joint_names_star_arm.yaml"
        )

    return arm_joint_names, gripper_joint_names


def generate_launch_description():
    description_dir = get_package_share_directory("star_arm")
    moveit_dir = get_package_share_directory("star_arm_movit_config")
    default_model_path = os.path.join(description_dir, "urdf", "star_arm.urdf.xacro")
    arm_joint_names, gripper_joint_names = load_joint_config()

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=default_model_path,
        description="Absolute path to the robot URDF or Xacro file.",
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
        .sensors_3d(file_path="config/sensors_3d.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": robot_description}
    moveit_config.trajectory_execution = {
        "moveit_manage_controllers": False,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": [
                "arm_controller",
                "gripper_controller",
            ],
            "arm_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": arm_joint_names,
            },
            "gripper_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": gripper_joint_names,
            },
        },
    }

    static_world_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/joint_states_stamped")],
        output="screen",
    )

    joint_state_stamp_relay_node = Node(
        package="trajectory_bridge",
        executable="joint_state_stamp_relay",
        name="joint_state_stamp_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/joint_states",
                "output_topic": "/joint_states_stamped",
                "always_stamp_now": True,
            }
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "trajectory_execution.allowed_execution_duration_scaling": 4.0,
                "trajectory_execution.allowed_goal_duration_margin": 12.0,
                "trajectory_execution.allowed_start_tolerance": 0.03,
            },
        ],
        remappings=[("/joint_states", "/joint_states_stamped")],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_dir, "config", "moveit_point.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    trajectory_bridge_node = Node(
        package="trajectory_bridge",
        executable="trajectory_bridge_node",
        name="trajectory_bridge",
        output="screen",
        parameters=[
            {
                "arm_action_name": "/arm_controller/follow_joint_trajectory",
                "arm_command_topic": "/arm_controller/joint_trajectory",
                "arm_joint_names": arm_joint_names,
                "arm_goal_joint_tolerance": 0.05,
                "arm_lock_joint_enabled": False,
                "arm_lock_joint_name": "joint5",
                "arm_lock_joint_position": 0.0,
                "arm_lock_joint_tolerance": 0.03,
                "gripper_action_name": "/gripper_controller/follow_joint_trajectory",
                "gripper_command_topic": "/gripper_controller/joint_trajectory",
                "gripper_joint_names": gripper_joint_names,
                "gripper_goal_joint_tolerance": 0.002,
                "joint_state_topic": "/joint_states_stamped",
                "execution_timeout_sec": 45.0,
                "joint_state_timeout_sec": 2.0,
                "settling_samples": 2,
                # 该值需与 ESP32 侧 app_config::kTrajectoryPointCapacity 保持一致。
                "use_last_point_only": False,
                # 要与 ESP32 侧 app_config::kTrajectoryPointCapacity 保持一致，且不宜设置过大，否则可能导致内存不足。
                "max_forward_points": 16,
                "min_point_interval_sec": 0.0,
            }
        ],
    )

    return LaunchDescription(
        [
            model_arg,
            static_world_tf,
            joint_state_stamp_relay_node,
            robot_state_publisher_node,
            move_group_node,
            rviz_node,
            trajectory_bridge_node,
        ]
    )
