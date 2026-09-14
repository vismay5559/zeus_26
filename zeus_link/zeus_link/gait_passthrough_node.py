"""
The simplest possible "policy": a zero residual at 250 Hz.

With enable false (the default) the board keeps receiving commands, so the link
stays healthy, but stays IDLE - nothing moves. That is the state to watch
sensors in.

With enable true the STM32 arms the drives and plays its stored gait with no
correction at all. That is the first thing to run on the full robot, and the
baseline any RL policy has to beat.

    ros2 run zeus_link gait_passthrough_node                      # watch only
    ros2 run zeus_link gait_passthrough_node --ros-args -p enable:=true

On start it sends enable false for HANDSHAKE_SEC before anything else. The
STM32 latches every fault and will not arm again until it has seen a command
with enable false - so without this, a node restarted after a fault with
enable:=true would be ignored indefinitely with nothing to say why. Starting a
node is an explicit act, which is exactly what that handshake is meant to
require.

It is also the template for a real policy: subscribe zeus/state with STATE_QOS,
publish zeus/command with COMMAND_QOS, do the handshake, then fill residual_rad
and enable.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from zeus_msgs.msg import NexusCommand, NexusState

from .nexus_proto import NUM_JOINTS, SAFETY_NAMES
from .qos import COMMAND_QOS, STATE_QOS

HANDSHAKE_SEC = 0.2


class GaitPassthroughNode(Node):

    def __init__(self) -> None:
        super().__init__("gait_passthrough")

        rate = float(self.declare_parameter("rate_hz", 250.0).value)
        self._enable = bool(self.declare_parameter("enable", False).value)

        self._pub = self.create_publisher(NexusCommand, "zeus/command", COMMAND_QOS)
        self.create_subscription(NexusState, "zeus/state", self._on_state, STATE_QOS)

        self._cmd = NexusCommand()
        self._cmd.residual_rad = [0.0] * NUM_JOINTS
        self._cmd.enable = False
        self._safety = None
        self._handshake_ticks = max(1, int(round(HANDSHAKE_SEC * rate)))

        self.create_timer(1.0 / rate, self._tick)

        if self._enable:
            self.get_logger().warn(
                f"ENABLED - zero residual at {rate:g} Hz. The robot will arm and "
                "walk the stored gait.")
        else:
            self.get_logger().info(
                f"zero residual at {rate:g} Hz with enable false: the board stays "
                "IDLE. Set -p enable:=true to walk the stored gait.")

    def _tick(self) -> None:
        if self._handshake_ticks > 0:
            self._handshake_ticks -= 1
            if self._handshake_ticks == 0:
                self._cmd.enable = self._enable
        self._cmd.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._cmd)

    def _on_state(self, msg: NexusState) -> None:
        if msg.safety_state != self._safety:
            self._safety = msg.safety_state
            name = SAFETY_NAMES.get(msg.safety_state, str(msg.safety_state))
            self.get_logger().info(f"board safety state: {name}")
            if name == "FAULT" and self._enable:
                self.get_logger().warn(
                    "the board faulted and has idled the drives. Restart this "
                    "node to re-arm - it sends the enable-false handshake first.")

    def stand_down(self) -> None:
        self._handshake_ticks = 0
        self._cmd.enable = False
        self._cmd.header.stamp = self.get_clock().now().to_msg()
        for _ in range(3):
            self._pub.publish(self._cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GaitPassthroughNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Only possible if the context is still up. On ctrl-c it is not, and
        # the robot is idled another way: commands stop, and 200 ms later the
        # STM32's link fault takes the drives - or sooner, if the link node is
        # shut down too, since it sends enable false on its way out.
        if rclpy.ok():
            node.stand_down()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
