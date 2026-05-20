#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState


class JointStateStampRelay(Node):
    def __init__(self):
        super().__init__('joint_state_stamp_relay')
        self.declare_parameters(
            '',
            [
                ('input_topic', '/joint_states'),
                ('output_topic', '/joint_states_stamped'),
                ('always_stamp_now', True),
            ],
        )

        self._input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self._output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self._always_stamp_now = self.get_parameter('always_stamp_now').get_parameter_value().bool_value
        self._warned_zero_stamp = False

        qos = QoSPresetProfiles.SENSOR_DATA.value
        self._publisher = self.create_publisher(JointState, self._output_topic, qos)
        self._subscription = self.create_subscription(
            JointState,
            self._input_topic,
            self._joint_state_callback,
            qos,
        )
        self.get_logger().info(
            'JointState stamp relay ready. '
            f'input={self._input_topic}, output={self._output_topic}, '
            f'always_stamp_now={self._always_stamp_now}'
        )

    def _joint_state_callback(self, msg: JointState):
        output = JointState()
        output.header = msg.header
        output.name = list(msg.name)
        output.position = list(msg.position)
        output.velocity = list(msg.velocity)
        output.effort = list(msg.effort)

        stamp_is_zero = (msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0)
        if self._always_stamp_now or stamp_is_zero:
            if stamp_is_zero and not self._warned_zero_stamp:
                self.get_logger().warning(
                    '检测到输入 joint_states 时间戳为 0，已改写为当前时间。'
                )
                self._warned_zero_stamp = True
            output.header.stamp = self.get_clock().now().to_msg()

        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateStampRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
