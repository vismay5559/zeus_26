"""
Hardcoded single-actuator validation test.

Sends the sequence:
    hold 0.0  (hold_zero_sec)
    → ramp to target  (hold_target_sec)
    → return to 0.0   (hold_zero_sec)

While running, subscribes to /dynamic_joint_states and prints
load_encoder_position and torque_estimate every second so you can
verify the hardware interface is receiving CAN telemetry from the ODrive.
"""

import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Float64MultiArray


class HardcodedActuatorTestNode(Node):
    def __init__(self):
        super().__init__('hardcoded_actuator_test_node')

        self.declare_parameter('command_topic',
                               '/single_actuator_command_controller/commands')
        self.declare_parameter('state_topic', '/dynamic_joint_states')
        self.declare_parameter('joint_name', 'joint_0')
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('hold_zero_sec', 2.0)
        self.declare_parameter('hold_target_sec', 2.0)
        self.declare_parameter('target', 0.02)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._cmd_topic   = self.get_parameter('command_topic').value
        self._state_topic = self.get_parameter('state_topic').value
        self._joint_name  = self.get_parameter('joint_name').value
        self._start_delay = float(self.get_parameter('start_delay_sec').value)
        self._hold_zero   = float(self.get_parameter('hold_zero_sec').value)
        self._hold_target = float(self.get_parameter('hold_target_sec').value)
        self._target      = float(self.get_parameter('target').value)
        self._rate        = float(self.get_parameter('publish_rate_hz').value)

        # Publisher: sends position commands
        self._pub = self.create_publisher(Float64MultiArray, self._cmd_topic, 10)

        # Subscriber: reads back ODrive telemetry
        self.create_subscription(
            DynamicJointState,
            self._state_topic,
            self._state_callback,
            10,
        )

        # Latest telemetry values (updated by subscription)
        self._load_enc_pos   = float('nan')
        self._torque_estimate = float('nan')
        self._after_spring   = float('nan')
        self._current_cmd    = 0.0

        self._start_time = self.get_clock().now()
        self._finished   = False

        # Command timer (at publish_rate_hz)
        self.create_timer(1.0 / self._rate, self._command_timer)

        # Print timer (1 Hz)
        self.create_timer(1.0, self._print_timer)

        self.get_logger().info(
            f'\n'
            f'  ┌─────────────────────────────────────────┐\n'
            f'  │  Single-Actuator Validation Test        │\n'
            f'  │  Joint     : {self._joint_name:<28}│\n'
            f'  │  Command   : {self._cmd_topic:<28}│\n'
            f'  │  Target    : {self._target:<28.4f}│\n'
            f'  │  Sequence  : 0 → {self._target:.4f} → 0              │\n'
            f'  └─────────────────────────────────────────┘'
        )

    # ------------------------------------------------------------------
    # Command timer — publishes position targets
    # ------------------------------------------------------------------

    def _command_timer(self):
        if self._finished:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        if elapsed < self._start_delay:
            return  # waiting for controllers to come up

        seq_time = elapsed - self._start_delay

        if seq_time < self._hold_zero:
            cmd = 0.0
        elif seq_time < self._hold_zero + self._hold_target:
            cmd = self._target
        elif seq_time < 2.0 * self._hold_zero + self._hold_target:
            cmd = 0.0
        else:
            self._publish(0.0)
            self.get_logger().info('Test complete. Final command: 0.0')
            self._finished = True
            return

        self._current_cmd = cmd
        self._publish(cmd)

    def _publish(self, value: float):
        msg = Float64MultiArray()
        msg.data = [value]
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # State subscriber — caches latest ODrive telemetry
    # ------------------------------------------------------------------

    def _state_callback(self, msg: DynamicJointState):
        for name, iface_vals in zip(msg.joint_names, msg.interface_values):
            if name != self._joint_name:
                continue
            for iface, val in zip(iface_vals.interface_names, iface_vals.values):
                if iface == 'load_encoder_position':
                    self._load_enc_pos = val
                elif iface == 'torque_estimate':
                    self._torque_estimate = val
                elif iface == 'after_spring_angle':
                    self._after_spring = val

    # ------------------------------------------------------------------
    # Print timer — logs telemetry at 1 Hz
    # ------------------------------------------------------------------

    def _print_timer(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

        # Determine current phase label
        seq_time = elapsed - self._start_delay
        if elapsed < self._start_delay:
            phase = 'WAITING (controllers starting)'
        elif seq_time < self._hold_zero:
            phase = 'HOLDING ZERO'
        elif seq_time < self._hold_zero + self._hold_target:
            phase = f'MOVING TO TARGET ({self._target:.4f})'
        elif seq_time < 2.0 * self._hold_zero + self._hold_target:
            phase = 'RETURNING TO ZERO'
        elif self._finished:
            phase = 'DONE'
        else:
            phase = 'DONE'

        self.get_logger().info(
            f'[{elapsed:6.1f}s]  phase={phase}\n'
            f'           cmd_sent       = {self._current_cmd:+.4f} rev\n'
            f'           load_enc_pos   = {self._load_enc_pos:+.4f} rev   '
            f'(from ODrive Get_Encoder_Estimates)\n'
            f'           torque_est     = {self._torque_estimate:+.4f} Nm    '
            f'(from ODrive Get_Torques)\n'
            f'           after_spring   = {self._after_spring:+.4f} rad  '
            f'(always nan in command_only_mode)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = HardcodedActuatorTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
