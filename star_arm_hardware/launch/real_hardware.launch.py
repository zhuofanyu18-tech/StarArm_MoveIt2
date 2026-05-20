"""
启动真实的 star_arm 硬件控制。

用法:
  ros2 launch star_arm_hardware real_hardware.launch.py serial_port:=/dev/ttyUSB0

流程:
  1. 加载 URDF（含 ros2_control 配置，使用真实硬件接口）
  2. 启动 robot_state_publisher
  3. 启动 controller_manager (ros2_control_node)
  4. 加载 joint_state_broadcaster（发布 /joint_states）
  5. 加载 arm_controller + gripper_controller（JointTrajectoryController）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── 声明参数 ──────────────────────────────────────────────
    serial_port = LaunchConfiguration("serial_port")
    use_rviz = LaunchConfiguration("use_rviz")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="舵机串口设备路径",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="false", description="是否启动 RViz"
    )

    # ── 机器人描述 (URDF) ────────────────────────────────────
    # 通过 xacro 生成 URDF，传入 use_mock:=false 和串口参数
    xacro_path = PathJoinSubstitution([
        FindPackageShare("star_arm_movit_config"),
        "config", "my_robot.urdf.xacro",
    ])
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        xacro_path, " ",
        "use_mock:=false", " ",
        "serial_port:=", serial_port,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ── Node 1: robot_state_publisher ────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description,
                    {"publish_frequency": 50.0}],
    )

    # ── Node 2: controller_manager ───────────────────────────
    # ros2_control_node 是 controller_manager 的载体进程
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

    # ── 加载 Controller ────────────────────────────────────────
    # joint_state_broadcaster: 把硬件接口的状态发布到 /joint_states
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "-c", "/controller_manager"],
    )

    # arm_controller: 控制 joint1~4
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "-c", "/controller_manager",
                   "--inactive"],  # 先不激活，确认安全后手动激活
    )

    # gripper_controller: 控制 end_joint1
    load_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "-c", "/controller_manager",
                   "--inactive"],
    )

    # ── Node 3 (可选): RViz ──────────────────────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("star_arm_movit_config"),
            "config", "moveit.rviz",
        ])],
    )

    return LaunchDescription([
        declare_serial_port,
        declare_use_rviz,
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_arm_controller,
        load_gripper_controller,
        rviz,
    ])
