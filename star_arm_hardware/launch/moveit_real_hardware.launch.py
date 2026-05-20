"""
完整的 MoveIt2 + ros2_control 硬件控制。

控制链路:
  MoveIt2 (move_group)
    ↓ /arm_controller/follow_joint_trajectory (action)
  joint_trajectory_controller
    ↓ ros2_control read()/write()
  StarArmHardwareInterface
    ↓ USB 串口 SyncWrite
  舵机

用法:
  ros2 launch star_arm_hardware moveit_real_hardware.launch.py serial_port:=/dev/ttyACM0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    use_rviz = LaunchConfiguration("use_rviz")
    use_mock = LaunchConfiguration("use_mock")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0",
        description="舵机串口设备路径",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="启动 RViz",
    )
    declare_use_mock = DeclareLaunchArgument(
        "use_mock", default_value="false",
        description="使用 mock 硬件 (仿真)",
    )

    # ── URDF 生成 ────────────────────────────────────────────
    xacro_path = PathJoinSubstitution([
        FindPackageShare("star_arm_movit_config"),
        "config", "my_robot.urdf.xacro",
    ])
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        xacro_path, " ",
        "use_mock:=", use_mock, " ",
        "serial_port:=", serial_port,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── MoveIt 配置 ──────────────────────────────────────────
    # 先用文件路径初始化 Builder，再用 to_moveit_configs() 得到 MoveItConfigs 对象
    # 然后覆盖 robot_description 为带参数的动态 URDF
    star_arm_dir = get_package_share_directory("star_arm")
    default_model_path = os.path.join(star_arm_dir, "urdf", "star_arm.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder("my_robot", package_name="star_arm_movit_config")
        .robot_description(file_path=default_model_path)
        .robot_description_semantic(file_path="config/my_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    # 覆盖 robot_description 为动态 URDF（带 xacro 参数）
    moveit_config.robot_description = robot_description
    # 覆盖 trajectory_execution 为 ros2_control 模式
    moveit_config.trajectory_execution = {
        "moveit_manage_controllers": False,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "moveit_simple_controller_manager": {
            "controller_names": ["arm_controller", "gripper_controller"],
            "arm_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ["joint1", "joint2", "joint3", "joint4"],
            },
            "gripper_controller": {
                "action_ns": "follow_joint_trajectory",
                "type": "FollowJointTrajectory",
                "default": True,
                "joints": ["end_joint1"],
            },
        },
    }

    # ── Node 1: robot_state_publisher ────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"publish_frequency": 50.0}],
    )

    # ── Node 2: controller_manager ───────────────────────────
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare("star_arm_movit_config"),
                "config", "ros2_controllers.yaml",
            ]),
        ],
        output="screen",
    )

    # ── Controller spawners ──────────────────────────────────
    spawn_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    # arm_controller + gripper_controller: 延迟 3 秒等 broadcaster 激活
    spawn_controllers = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "-c", "/controller_manager", "--activate"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["gripper_controller", "-c", "/controller_manager", "--activate"],
            ),
        ],
    )

    # ── Node 3: move_group ───────────────────────────────────
    # moveit_config 是 MoveItConfigs 对象，传参时必须 .to_dict()
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": False,
                "trajectory_execution.allowed_execution_duration_scaling": 4.0,
                "trajectory_execution.allowed_goal_duration_margin": 12.0,
                "trajectory_execution.allowed_start_tolerance": 0.03,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ── Node 4: RViz ─────────────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("star_arm_movit_config"),
            "config", "moveit.rviz",
        ])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        declare_serial_port,
        declare_use_rviz,
        declare_use_mock,
        robot_state_publisher,
        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_controllers,
        move_group,
        rviz,
    ])
