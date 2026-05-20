#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

int main(int argc, char * argv[])
{
  // 1. 初始化 ROS 2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  // MoveIt 需要自动声明参数
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("star_arm_grab_node", node_options);

  // 2. 启动一个后台线程来处理 ROS 回调机制，防止 MoveIt 动作死锁
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // 3. 初始化 MoveIt 的 MoveGroupInterface
  // ！！注意：请将 "arm" 替换为你实际在 MoveIt Setup Assistant 中设置的规划组名称！！
  // 你可以在 star_arm_movit_config/config/my_robot.srdf 中找到它
  static const std::string ARM_PLANNING_GROUP = "arm";
  static const std::string GRIPPER_PLANNING_GROUP = "gripper";
  static const std::string BASE_FRAME = "base_link";
  static const std::string TCP_LINK = "tcp_link";
  static constexpr double GRIPPER_GOAL_JOINT_TOLERANCE = 0.002;
  moveit::planning_interface::MoveGroupInterface arm_move_group(
    move_group_node, ARM_PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_move_group(
    move_group_node, GRIPPER_PLANNING_GROUP);

  const auto get_or_declare_bool =
    [&move_group_node](const std::string & name, bool default_value) {
      if (!move_group_node->has_parameter(name)) {
        move_group_node->declare_parameter<bool>(name, default_value);
      }
      bool value = default_value;
      move_group_node->get_parameter(name, value);
      return value;
    };
  const auto get_or_declare_double =
    [&move_group_node](const std::string & name, double default_value) {
      if (!move_group_node->has_parameter(name)) {
        move_group_node->declare_parameter<double>(name, default_value);
      }
      double value = default_value;
      move_group_node->get_parameter(name, value);
      return value;
    };
  const auto get_or_declare_int =
    [&move_group_node](const std::string & name, int default_value) {
      if (!move_group_node->has_parameter(name)) {
        move_group_node->declare_parameter<int>(name, default_value);
      }
      int value = default_value;
      move_group_node->get_parameter(name, value);
      return value;
    };
  const auto get_or_declare_string =
    [&move_group_node](const std::string & name, const std::string & default_value) {
      if (!move_group_node->has_parameter(name)) {
        move_group_node->declare_parameter<std::string>(name, default_value);
      }
      std::string value = default_value;
      move_group_node->get_parameter(name, value);
      return value;
    };

  const bool use_position_only_goal = get_or_declare_bool("use_position_only_goal", true);
  const double planning_time_sec = get_or_declare_double("planning_time_sec", 10.0);
  const int planning_attempts = get_or_declare_int("planning_attempts", 10);
  const double goal_position_tolerance =
    get_or_declare_double("goal_position_tolerance", 0.005);
  const double goal_orientation_tolerance =
    get_or_declare_double("goal_orientation_tolerance", 0.2);
  const bool lock_joint5 = get_or_declare_bool("lock_joint5", false);
  const std::string locked_joint_name = get_or_declare_string("locked_joint_name", "joint5");
  const double locked_joint_position_rad =
    get_or_declare_double("locked_joint_position_rad", 0.0);
  const double lock_joint_tolerance_rad =
    get_or_declare_double("lock_joint_tolerance_rad", 0.03);
  const double grab_pose_z_offset_m =
    get_or_declare_double("grab_pose_z_offset_m", 0.03);
  const std::string place_named_target =
    get_or_declare_string("place_named_target", "fangzhi");
  const std::string detect_named_target =
    get_or_declare_string("detect_named_target", "detect");
  const std::string gripper_close_named_target =
    get_or_declare_string("gripper_close_named_target", "gripper_close");
  const std::string gripper_open_named_target =
    get_or_declare_string("gripper_open_named_target", "gripper_open");

  // 设置最大速度和加速度的缩放比例（0.0 到 1.0 之间）
  arm_move_group.setMaxVelocityScalingFactor(0.8);
  arm_move_group.setMaxAccelerationScalingFactor(0.8);
  arm_move_group.setPlanningTime(planning_time_sec);
  arm_move_group.setNumPlanningAttempts(planning_attempts);
  arm_move_group.setGoalPositionTolerance(goal_position_tolerance);
  arm_move_group.setGoalOrientationTolerance(goal_orientation_tolerance);
  arm_move_group.setPoseReferenceFrame(BASE_FRAME);
  if (!arm_move_group.setEndEffectorLink(TCP_LINK)) {
    RCLCPP_WARN(
      move_group_node->get_logger(),
      "无法将末端执行器设置为 '%s'，当前链接为 '%s'。请确认 SRDF/URDF 已包含 tcp_link。",
      TCP_LINK.c_str(), arm_move_group.getEndEffectorLink().c_str());
  }
  gripper_move_group.setMaxVelocityScalingFactor(1.0);
  gripper_move_group.setMaxAccelerationScalingFactor(1.0);
  gripper_move_group.setPlanningTime(planning_time_sec);
  gripper_move_group.setNumPlanningAttempts(planning_attempts);
  gripper_move_group.setGoalJointTolerance(GRIPPER_GOAL_JOINT_TOLERANCE);
  RCLCPP_INFO(
    move_group_node->get_logger(),
    "Arm MoveGroup 已就绪: planning_frame=%s, pose_reference_frame=%s, end_effector_link=%s, use_position_only_goal=%s, lock_joint5=%s(%s=%.4f rad, tol=%.4f rad)",
    arm_move_group.getPlanningFrame().c_str(),
    arm_move_group.getPoseReferenceFrame().c_str(),
    arm_move_group.getEndEffectorLink().c_str(),
    use_position_only_goal ? "true" : "false",
    lock_joint5 ? "true" : "false",
    locked_joint_name.c_str(),
    locked_joint_position_rad,
    lock_joint_tolerance_rad);
  RCLCPP_INFO(
    move_group_node->get_logger(),
    "Gripper MoveGroup 已就绪: planning_frame=%s, joint_tolerance=%.4f rad",
    gripper_move_group.getPlanningFrame().c_str(),
    GRIPPER_GOAL_JOINT_TOLERANCE);
  RCLCPP_INFO(
    move_group_node->get_logger(),
    "抓取流程参数: grab_pose_z_offset_m=%.4f, place_named_target=%s, detect_named_target=%s, gripper_close_named_target=%s, gripper_open_named_target=%s",
    grab_pose_z_offset_m,
    place_named_target.c_str(),
    detect_named_target.c_str(),
    gripper_close_named_target.c_str(),
    gripper_open_named_target.c_str());

  auto execution_busy_pub =
    move_group_node->create_publisher<std_msgs::msg::Bool>("/grab/execution_busy", 10);
  auto publish_execution_busy = [execution_busy_pub](bool busy) {
      std_msgs::msg::Bool msg;
      msg.data = busy;
      execution_busy_pub->publish(msg);
    };
  publish_execution_busy(false);

  // 4. 创建话题订阅者，订阅 /grab_pose
  auto pose_sub = move_group_node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/grab_pose", 10,
    [&arm_move_group, &gripper_move_group, move_group_node, use_position_only_goal, lock_joint5,
      locked_joint_name, locked_joint_position_rad, lock_joint_tolerance_rad,
      grab_pose_z_offset_m, place_named_target, detect_named_target,
      gripper_close_named_target, gripper_open_named_target,
      publish_execution_busy](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      auto log_wait_next_goal = [&move_group_node]() {
        RCLCPP_INFO(move_group_node->get_logger(), "等待下一次接收抓取坐标...");
      };
      auto clear_arm_request_state = [&arm_move_group]() {
        arm_move_group.clearPoseTargets();
        arm_move_group.clearPathConstraints();
      };
      auto clear_gripper_request_state = [&gripper_move_group]() {
        gripper_move_group.clearPoseTargets();
        gripper_move_group.clearPathConstraints();
      };
      auto finish_request = [&]() {
        clear_arm_request_state();
        clear_gripper_request_state();
        publish_execution_busy(false);
        log_wait_next_goal();
      };
      auto execute_arm_named_target =
        [&](const std::string & named_target, const std::string & step_name) {
          clear_arm_request_state();
          arm_move_group.setStartStateToCurrentState();
          if (!arm_move_group.setNamedTarget(named_target)) {
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "无法设置机械臂命名动作 '%s'（步骤：%s）。",
              named_target.c_str(), step_name.c_str());
            clear_arm_request_state();
            return false;
          }

          try {
            auto move_result = arm_move_group.move();
            clear_arm_request_state();
            if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(
                move_group_node->get_logger(),
                "%s 成功（named target=%s）。",
                step_name.c_str(), named_target.c_str());
              return true;
            }

            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "%s 失败（named target=%s, error_code=%d）。",
              step_name.c_str(), named_target.c_str(), move_result.val);
          } catch (const std::exception & exc) {
            clear_arm_request_state();
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "%s 抛出异常：%s",
              step_name.c_str(), exc.what());
            return false;
          } catch (...) {
            clear_arm_request_state();
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "%s 抛出未知异常。",
              step_name.c_str());
            return false;
          }

          clear_arm_request_state();
          return false;
        };
      auto execute_gripper_named_target =
        [&](const std::string & named_target, const std::string & step_name) {
          clear_gripper_request_state();
          gripper_move_group.setStartStateToCurrentState();
          if (!gripper_move_group.setNamedTarget(named_target)) {
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "无法设置夹爪命名动作 '%s'（步骤：%s）。",
              named_target.c_str(), step_name.c_str());
            clear_gripper_request_state();
            return false;
          }

          try {
            auto move_result = gripper_move_group.move();
            clear_gripper_request_state();
            if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(
                move_group_node->get_logger(),
                "%s 成功（named target=%s）。",
                step_name.c_str(), named_target.c_str());
              return true;
            }

            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "%s 失败（named target=%s, error_code=%d）。",
              step_name.c_str(), named_target.c_str(), move_result.val);
          } catch (const std::exception & exc) {
            clear_gripper_request_state();
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "%s 抛出异常：%s",
              step_name.c_str(), exc.what());
            return false;
          } catch (...) {
            clear_gripper_request_state();
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "%s 抛出未知异常。",
              step_name.c_str());
            return false;
          }

          clear_gripper_request_state();
          return false;
        };
      auto execute_grab_pose_target = [&](geometry_msgs::msg::PoseStamped & target) {
          arm_move_group.setStartStateToCurrentState();
          clear_arm_request_state();

          if (lock_joint5) {
            const auto current_joint_names = arm_move_group.getJointNames();
            const auto joint_iter =
              std::find(current_joint_names.begin(), current_joint_names.end(), locked_joint_name);
            if (joint_iter == current_joint_names.end()) {
              RCLCPP_WARN(
                move_group_node->get_logger(),
                "规划组中不存在关节 %s；将继续规划。",
                locked_joint_name.c_str());
            } else {
              moveit_msgs::msg::Constraints path_constraints;
              moveit_msgs::msg::JointConstraint joint_constraint;
              joint_constraint.joint_name = locked_joint_name;
              joint_constraint.position = locked_joint_position_rad;
              joint_constraint.tolerance_above = lock_joint_tolerance_rad;
              joint_constraint.tolerance_below = lock_joint_tolerance_rad;
              joint_constraint.weight = 1.0;
              path_constraints.joint_constraints.push_back(joint_constraint);
              arm_move_group.setPathConstraints(path_constraints);
              RCLCPP_INFO(
                move_group_node->get_logger(),
                "已锁定关节 %s 为 %.4f rad（容差 ±%.4f rad）进行抓取位规划。",
                locked_joint_name.c_str(),
                locked_joint_position_rad,
                lock_joint_tolerance_rad);
            }
          } else {
            RCLCPP_DEBUG(
              move_group_node->get_logger(),
              "未启用关节锁定（lock_joint5=false）。");
          }

          if (use_position_only_goal) {
            if (!arm_move_group.setPositionTarget(
                  target.pose.position.x,
                  target.pose.position.y,
                  target.pose.position.z,
                  TCP_LINK)) {
              RCLCPP_ERROR(
                move_group_node->get_logger(),
                "设置抓取位 Position 目标失败（link=%s, frame=%s）。",
                TCP_LINK.c_str(), target.header.frame_id.c_str());
              clear_arm_request_state();
              return false;
            }
          } else {
            const double q_norm_sq =
              target.pose.orientation.x * target.pose.orientation.x +
              target.pose.orientation.y * target.pose.orientation.y +
              target.pose.orientation.z * target.pose.orientation.z +
              target.pose.orientation.w * target.pose.orientation.w;
            if (q_norm_sq < 1e-12) {
              target.pose.orientation.x = 0.0;
              target.pose.orientation.y = 0.0;
              target.pose.orientation.z = 0.0;
              target.pose.orientation.w = 1.0;
              RCLCPP_WARN(
                move_group_node->get_logger(),
                "收到非法四元数（全 0），已替换为单位四元数。");
            } else if (std::fabs(q_norm_sq - 1.0) > 1e-6) {
              const double q_norm = std::sqrt(q_norm_sq);
              target.pose.orientation.x /= q_norm;
              target.pose.orientation.y /= q_norm;
              target.pose.orientation.z /= q_norm;
              target.pose.orientation.w /= q_norm;
            }

            if (!arm_move_group.setPoseTarget(target, TCP_LINK)) {
              RCLCPP_ERROR(
                move_group_node->get_logger(),
                "设置抓取位 Pose 目标失败（link=%s, frame=%s）。",
                TCP_LINK.c_str(), target.header.frame_id.c_str());
              clear_arm_request_state();
              return false;
            }
          }

          try {
            auto move_result = arm_move_group.move();
            clear_arm_request_state();
            if (move_result == moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_INFO(move_group_node->get_logger(), "机械臂已到达抓取位。");
              return true;
            }

            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "机械臂到抓取位失败（error_code=%d）。请检查目标点是否在工作空间内，或者是否有碰撞/奇异点。",
              move_result.val);
          } catch (const std::exception & exc) {
            clear_arm_request_state();
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "机械臂到抓取位抛出异常：%s",
              exc.what());
            return false;
          } catch (...) {
            clear_arm_request_state();
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "机械臂到抓取位抛出未知异常。");
            return false;
          }

          clear_arm_request_state();
          return false;
        };
      auto attempt_return_to_detect = [&]() {
          RCLCPP_INFO(
            move_group_node->get_logger(),
            "抓取位步骤失败，开始尝试回到检测位（named target=%s）。",
            detect_named_target.c_str());
          if (!execute_arm_named_target(detect_named_target, "机械臂回到检测位")) {
            RCLCPP_ERROR(
              move_group_node->get_logger(),
              "回到检测位失败，请人工检查机械臂状态。");
          }
        };

      auto target = *msg;
      if (target.header.frame_id.empty()) {
        target.header.frame_id = BASE_FRAME;
        RCLCPP_WARN(
          move_group_node->get_logger(),
          "/grab_pose 的 frame_id 为空，已按 '%s' 处理。", BASE_FRAME.c_str());
      }
      if (target.header.frame_id != BASE_FRAME && target.header.frame_id != "/" + BASE_FRAME) {
        RCLCPP_WARN(
          move_group_node->get_logger(),
          "收到目标 frame_id=%s（建议直接使用 base_link）。如果 TF 不可达会规划失败。",
          target.header.frame_id.c_str());
      }

      RCLCPP_INFO(
        move_group_node->get_logger(),
        "收到抓取目标，tcp_link 目标(frame=%s): p=[%.4f, %.4f, %.4f], q=[%.4f, %.4f, %.4f, %.4f]",
        target.header.frame_id.c_str(),
        target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.x, target.pose.orientation.y,
        target.pose.orientation.z, target.pose.orientation.w);
      target.pose.position.z += grab_pose_z_offset_m;
      RCLCPP_INFO(
        move_group_node->get_logger(),
        "抓取高度补偿后目标(frame=%s): p=[%.4f, %.4f, %.4f]，z_offset=%.4f m",
        target.header.frame_id.c_str(),
        target.pose.position.x, target.pose.position.y, target.pose.position.z,
        grab_pose_z_offset_m);

      publish_execution_busy(true);
      if (!execute_grab_pose_target(target)) {
        attempt_return_to_detect();
        finish_request();
        return;
      }
      if (!execute_gripper_named_target(gripper_close_named_target, "夹爪闭合")) {
        finish_request();
        return;
      }
      if (!execute_arm_named_target(place_named_target, "机械臂执行放置动作")) {
        finish_request();
        return;
      }
      if (!execute_gripper_named_target(gripper_open_named_target, "夹爪张开")) {
        finish_request();
        return;
      }
      if (!execute_arm_named_target(detect_named_target, "机械臂回到检测位")) {
        finish_request();
        return;
      }

      finish_request();
    });

  RCLCPP_INFO(move_group_node->get_logger(), "抓取节点已启动，正在等待 /grab_pose 话题...");

  // 5. 等待后台线程结束（保持节点运行）
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
