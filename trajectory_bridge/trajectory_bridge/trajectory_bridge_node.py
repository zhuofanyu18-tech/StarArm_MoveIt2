#!/usr/bin/env python3
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryValidationError(ValueError):
    def __init__(self, message, error_code):
        super().__init__(message)
        self.error_code = error_code


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge_node')

        self.declare_parameters(
            '',
            [
                ('arm_action_name', '/arm_controller/follow_joint_trajectory'),
                ('arm_command_topic', '/arm_controller/joint_trajectory'),
                ('arm_joint_names', Parameter.Type.STRING_ARRAY),
                ('arm_goal_joint_tolerance', 0.04),
                ('arm_lock_joint_enabled', False),
                ('arm_lock_joint_name', 'joint5'),
                ('arm_lock_joint_position', 0.0),
                ('arm_lock_joint_tolerance', 0.03),
                ('gripper_action_name', '/gripper_controller/follow_joint_trajectory'),
                ('gripper_command_topic', '/gripper_controller/joint_trajectory'),
                ('gripper_joint_names', Parameter.Type.STRING_ARRAY),
                ('gripper_goal_joint_tolerance', 0.002),
                ('joint_state_topic', '/joint_states'),
                ('execution_timeout_sec', 20.0),
                ('joint_state_timeout_sec', 0.5),
                ('settling_samples', 3),
                ('use_last_point_only', False),
                ('max_forward_points', 16),
                ('min_point_interval_sec', 0.0),
            ],
        )

        self.arm_action_name = self.get_parameter('arm_action_name').get_parameter_value().string_value
        self.arm_command_topic = self.get_parameter('arm_command_topic').get_parameter_value().string_value
        self.arm_joint_names = list(self.get_parameter('arm_joint_names').value or [])
        self.arm_goal_joint_tolerance = float(self.get_parameter('arm_goal_joint_tolerance').value)
        self.arm_lock_joint_enabled = bool(self.get_parameter('arm_lock_joint_enabled').value)
        self.arm_lock_joint_name = self.get_parameter('arm_lock_joint_name').value
        self.arm_lock_joint_position = float(self.get_parameter('arm_lock_joint_position').value)
        self.arm_lock_joint_tolerance = float(self.get_parameter('arm_lock_joint_tolerance').value)
        self.gripper_action_name = self.get_parameter('gripper_action_name').get_parameter_value().string_value
        self.gripper_command_topic = self.get_parameter('gripper_command_topic').get_parameter_value().string_value
        self.gripper_joint_names = list(self.get_parameter('gripper_joint_names').value or [])
        self.gripper_goal_joint_tolerance = float(
            self.get_parameter('gripper_goal_joint_tolerance').value
        )
        self.use_last_point_only = self.get_parameter('use_last_point_only').get_parameter_value().bool_value

        self._action_callback_group = ReentrantCallbackGroup()
        self._telemetry_callback_group = ReentrantCallbackGroup()

        self._joint_state_condition = threading.Condition()
        self._latest_joint_positions = {}
        self._latest_joint_state_monotonic = None

        self.arm_pub = self.create_publisher(JointTrajectory, self.arm_command_topic, 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, self.gripper_command_topic, 10)
        self.create_subscription(
            JointState,
            self.get_parameter('joint_state_topic').get_parameter_value().string_value,
            self._joint_state_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
            callback_group=self._telemetry_callback_group,
        )

        self.arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.arm_action_name,
            self.arm_execute_callback,
            callback_group=self._action_callback_group,
        )
        self.gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.gripper_action_name,
            self.gripper_execute_callback,
            callback_group=self._action_callback_group,
        )
        self.get_logger().info(
            'Trajectory bridge ready. '
            f'arm_action={self.arm_action_name}, arm_topic={self.arm_command_topic}, '
            f'gripper_action={self.gripper_action_name}, gripper_topic={self.gripper_command_topic}, '
            f'use_last_point_only={self.use_last_point_only}'
        )

    def _joint_state_callback(self, msg):
        snapshot = {}
        for joint_name, position in zip(msg.name, msg.position):
            snapshot[joint_name] = position

        with self._joint_state_condition:
            self._latest_joint_positions = snapshot
            self._latest_joint_state_monotonic = time.monotonic()
            self._joint_state_condition.notify_all()

    def arm_execute_callback(self, goal_handle):
        lock_joint_constraint = None
        if self.arm_lock_joint_enabled and self.arm_lock_joint_name:
            lock_joint_constraint = {
                'joint_name': self.arm_lock_joint_name,
                'target_position': self.arm_lock_joint_position,
                'tolerance': max(0.0, self.arm_lock_joint_tolerance),
            }
        return self._execute_goal(
            goal_handle=goal_handle,
            publisher=self.arm_pub,
            command_topic=self.arm_command_topic,
            expected_joint_names=self.arm_joint_names,
            goal_tolerance=self.arm_goal_joint_tolerance,
            target_name='机械臂',
            lock_joint_constraint=lock_joint_constraint,
        )

    def gripper_execute_callback(self, goal_handle):
        return self._execute_goal(
            goal_handle=goal_handle,
            publisher=self.gripper_pub,
            command_topic=self.gripper_command_topic,
            expected_joint_names=self.gripper_joint_names,
            goal_tolerance=self.gripper_goal_joint_tolerance,
            target_name='夹爪',
            lock_joint_constraint=None,
        )

    def _execute_goal(
        self,
        goal_handle,
        publisher,
        command_topic,
        expected_joint_names,
        goal_tolerance,
        target_name,
        lock_joint_constraint,
    ):
        self.get_logger().info(f'接收到{target_name}轨迹目标')
        try:
            forwarded = self._simplify_trajectory(
                goal_handle.request.trajectory,
                expected_joint_names=expected_joint_names,
            )
            if lock_joint_constraint:
                self._validate_locked_joint_positions(
                    trajectory=forwarded,
                    lock_joint_constraint=lock_joint_constraint,
                    target_name=target_name,
                )
            publisher.publish(forwarded)
            self.get_logger().info(
                f'已将{target_name}目标转发到 {command_topic}: '
                f'{len(forwarded.points)} point(s), joints={forwarded.joint_names}'
            )

            final_point = forwarded.points[-1]
            final_target = dict(zip(forwarded.joint_names, final_point.positions))
            wait_error = self._wait_for_goal_reached(final_target, goal_tolerance)
            if wait_error is not None:
                self.get_logger().error(f"{target_name}执行判定失败: {wait_error}")
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = wait_error
                return result

            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            return result
        except TrajectoryValidationError as exc:
            self.get_logger().error(f'{target_name}轨迹无效: {exc}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = exc.error_code
            result.error_string = str(exc)
            return result
        except Exception as exc:
            self.get_logger().error(f'处理{target_name}目标时发生异常: {exc}')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            return result

    def _wait_for_goal_reached(self, target_positions, goal_tolerance):
        timeout_sec = float(self.get_parameter('execution_timeout_sec').value)
        joint_state_timeout_sec = float(self.get_parameter('joint_state_timeout_sec').value)
        settling_samples = max(1, int(self.get_parameter('settling_samples').value))
        deadline = time.monotonic() + timeout_sec
        stable_samples = 0
        last_error_summary = '尚未收到 joint_states'

        while time.monotonic() < deadline:
            wait_timeout = min(0.05, max(0.0, deadline - time.monotonic()))
            with self._joint_state_condition:
                self._joint_state_condition.wait(timeout=wait_timeout)
                latest_positions = dict(self._latest_joint_positions)
                latest_stamp = self._latest_joint_state_monotonic

            if latest_stamp is None:
                stable_samples = 0
                continue

            age_sec = time.monotonic() - latest_stamp
            if age_sec > joint_state_timeout_sec:
                stable_samples = 0
                last_error_summary = (
                    f'joint_states 已过期 {age_sec:.2f}s，超过阈值 {joint_state_timeout_sec:.2f}s'
                )
                continue

            missing_joints = [
                joint_name for joint_name in target_positions if joint_name not in latest_positions
            ]
            if missing_joints:
                stable_samples = 0
                last_error_summary = f'joint_states 缺少关节: {missing_joints}'
                continue

            max_error = 0.0
            max_error_joint = None
            for joint_name, target_position in target_positions.items():
                error = abs(latest_positions[joint_name] - target_position)
                if error > max_error:
                    max_error = error
                    max_error_joint = joint_name

            if max_error <= goal_tolerance:
                stable_samples += 1
                if stable_samples >= settling_samples:
                    return None
            else:
                stable_samples = 0
                last_error_summary = (
                    f'关节 {max_error_joint} 误差 {max_error:.4f} 超过容差 {goal_tolerance:.4f}'
                )

        return f'等待执行到位超时: {last_error_summary}'

    def _validate_locked_joint_positions(self, trajectory, lock_joint_constraint, target_name):
        if not trajectory.points:
            return

        joint_name = lock_joint_constraint['joint_name']
        locked_position = float(lock_joint_constraint['target_position'])
        tolerance = max(0.0, float(lock_joint_constraint['tolerance']))

        if joint_name not in trajectory.joint_names:
            raise TrajectoryValidationError(
                f'{target_name}轨迹不包含锁定关节 {joint_name}，拒绝执行。',
                FollowJointTrajectory.Result.INVALID_JOINTS,
            )

        joint_index = trajectory.joint_names.index(joint_name)
        max_error = 0.0
        max_error_point_index = 0

        for point_index, point in enumerate(trajectory.points):
            if len(point.positions) <= joint_index:
                raise TrajectoryValidationError(
                    f'{target_name}轨迹点 {point_index} 缺少关节 {joint_name} 的位置数据。',
                    FollowJointTrajectory.Result.INVALID_GOAL,
                )
            error = abs(point.positions[joint_index] - locked_position)
            if error > max_error:
                max_error = error
                max_error_point_index = point_index

        if max_error > tolerance:
            raise TrajectoryValidationError(
                f'{target_name}轨迹违反锁定约束: {joint_name} 目标={locked_position:.4f}rad, '
                f'最大偏差={max_error:.4f}rad(点#{max_error_point_index}), 容差={tolerance:.4f}rad。',
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        self.get_logger().info(
            f'{target_name}轨迹通过锁定约束校验: {joint_name}={locked_position:.4f}rad '
            f'(容差 ±{tolerance:.4f}rad)'
        )

    def _simplify_trajectory(self, traj_msg, expected_joint_names):
        if len(traj_msg.points) == 0:
            raise TrajectoryValidationError(
                '接收到的轨迹没有点',
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        joint_names = self._resolve_joint_names(
            received_joint_names=list(traj_msg.joint_names),
            expected_joint_names=list(expected_joint_names or []),
        )
        source_joint_names = list(traj_msg.joint_names) or list(joint_names)
        selected_points = self._select_forward_points(list(traj_msg.points))

        simplified = JointTrajectory()
        simplified.header = traj_msg.header
        simplified.joint_names = joint_names

        last_time_sec = -1.0
        for index, source_point in enumerate(selected_points):
            point = JointTrajectoryPoint()
            point.positions = self._remap_positions(
                values=source_point.positions,
                source_joint_names=source_joint_names,
                target_joint_names=joint_names,
            )

            point_time_sec = self._duration_to_seconds(source_point.time_from_start)
            if point_time_sec < 0.0:
                point_time_sec = 0.0
            if point_time_sec <= last_time_sec:
                point_time_sec = last_time_sec + 0.05
            point.time_from_start.sec = int(point_time_sec)
            point.time_from_start.nanosec = int(round((point_time_sec % 1.0) * 1e9))
            if point.time_from_start.nanosec >= 1_000_000_000:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec -= 1_000_000_000

            last_time_sec = point_time_sec
            simplified.points.append(point)

        if not simplified.points:
            raise TrajectoryValidationError(
                '筛选后没有可转发的轨迹点。',
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        return simplified

    def _select_forward_points(self, points):
        if self.use_last_point_only or len(points) <= 1:
            return [points[-1]]

        min_interval_sec = max(0.0, float(self.get_parameter('min_point_interval_sec').value))
        max_forward_points = max(1, int(self.get_parameter('max_forward_points').value))
        if min_interval_sec <= 0.0 and len(points) <= max_forward_points:
            return points

        candidates = []

        for index, point in enumerate(points):
            is_last = index == len(points) - 1
            point_time_sec = self._duration_to_seconds(point.time_from_start)

            if not candidates:
                candidates.append(point)
                continue

            last_time_sec = self._duration_to_seconds(candidates[-1].time_from_start)
            if is_last or (point_time_sec - last_time_sec) >= min_interval_sec:
                candidates.append(point)

        if not candidates:
            candidates = [points[-1]]

        if len(candidates) <= max_forward_points:
            return candidates

        if max_forward_points == 1:
            return [candidates[-1]]

        selected = [candidates[0]]
        last_selected_index = 0
        max_index = len(candidates) - 1
        interior_slots = max_forward_points - 2

        for slot in range(1, interior_slots + 1):
            raw_index = round(slot * max_index / (interior_slots + 1))
            candidate_index = min(max_index - 1, max(last_selected_index + 1, raw_index))
            selected.append(candidates[candidate_index])
            last_selected_index = candidate_index

        selected.append(candidates[-1])
        return selected

    def _resolve_joint_names(self, received_joint_names, expected_joint_names):
        if expected_joint_names and len(set(expected_joint_names)) != len(expected_joint_names):
            raise TrajectoryValidationError(
                f'期望关节列表存在重复项: {expected_joint_names}',
                FollowJointTrajectory.Result.INVALID_JOINTS,
            )
        if received_joint_names and len(set(received_joint_names)) != len(received_joint_names):
            raise TrajectoryValidationError(
                f'收到的关节列表存在重复项: {received_joint_names}',
                FollowJointTrajectory.Result.INVALID_JOINTS,
            )

        if expected_joint_names:
            if not received_joint_names:
                raise TrajectoryValidationError(
                    '轨迹缺少 joint_names，无法与控制器关节顺序对齐。',
                    FollowJointTrajectory.Result.INVALID_JOINTS,
                )
            if received_joint_names == expected_joint_names:
                return expected_joint_names

            missing = [name for name in expected_joint_names if name not in received_joint_names]
            unexpected = [name for name in received_joint_names if name not in expected_joint_names]
            if missing or unexpected or len(received_joint_names) != len(expected_joint_names):
                raise TrajectoryValidationError(
                    f'轨迹关节与控制器不匹配。收到={received_joint_names}，期望={expected_joint_names}，'
                    f'缺失={missing}，额外={unexpected}',
                    FollowJointTrajectory.Result.INVALID_JOINTS,
                )

            self.get_logger().warning(
                f'收到的关节顺序为 {received_joint_names}，期望顺序为 {expected_joint_names}。'
                '已按控制器顺序自动重排轨迹点。'
            )
            return expected_joint_names

        if not received_joint_names:
            raise TrajectoryValidationError(
                '轨迹缺少 joint_names。',
                FollowJointTrajectory.Result.INVALID_JOINTS,
            )

        return received_joint_names

    def _remap_positions(self, values, source_joint_names, target_joint_names):
        if not values:
            raise TrajectoryValidationError(
                '轨迹点缺少 positions 数据。',
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        normalized = list(values)
        if len(normalized) != len(source_joint_names):
            raise TrajectoryValidationError(
                f'positions 长度 {len(normalized)} 与 joint_names 数量 '
                f'{len(source_joint_names)} 不一致。',
                FollowJointTrajectory.Result.INVALID_GOAL,
            )

        if source_joint_names == target_joint_names:
            return normalized

        value_by_joint = dict(zip(source_joint_names, normalized))
        return [value_by_joint[joint_name] for joint_name in target_joint_names]

    @staticmethod
    def _duration_to_seconds(duration_msg):
        return float(duration_msg.sec) + float(duration_msg.nanosec) / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
