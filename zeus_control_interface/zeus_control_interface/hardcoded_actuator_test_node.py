import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class HardcodedActuatorTestNode(Node):
    def __init__(self):
        super().__init__('hardcoded_actuator_test_node')

        self.declare_parameter(
            'command_topic',
            '/single_actuator_command_controller/commands',
        )
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('hold_zero_sec', 2.0)
        self.declare_parameter('hold_target_sec', 2.0)
        self.declare_parameter('target', 0.02)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.hold_zero_sec = float(self.get_parameter('hold_zero_sec').value)
        self.hold_target_sec = float(self.get_parameter('hold_target_sec').value)
        self.target = float(self.get_parameter('target').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.publisher = self.create_publisher(
            Float64MultiArray,
            self.command_topic,
            10,
        )

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.timer_callback)
        self.finished = False

        self.get_logger().info(
            f'Publishing hardcoded actuator sequence to {self.command_topic}: '
            f'0.0 -> {self.target} -> 0.0'
        )

    def timer_callback(self):
        if self.finished:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        if elapsed < self.start_delay_sec:
            return

        sequence_time = elapsed - self.start_delay_sec
        if sequence_time < self.hold_zero_sec:
            command = 0.0
        elif sequence_time < self.hold_zero_sec + self.hold_target_sec:
            command = self.target
        elif sequence_time < (2.0 * self.hold_zero_sec) + self.hold_target_sec:
            command = 0.0
        else:
            self.publish_command(0.0)
            self.get_logger().info('Hardcoded actuator test complete. Final command is 0.0.')
            self.finished = True
            return

        self.publish_command(command)

    def publish_command(self, command):
        message = Float64MultiArray()
        message.data = [command]
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = HardcodedActuatorTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
