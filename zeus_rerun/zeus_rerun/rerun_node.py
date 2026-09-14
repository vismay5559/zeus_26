"""
ROS node: /zeus/state and /zeus/command -> Rerun.

Use this whenever the link node is running (it holds the serial port, so the
serial tool cannot). Runs on the Pi; the viewer runs on your laptop.

    # laptop - start the viewer first
    rerun

    # Pi
    ros2 run zeus_rerun rerun_node --ros-args -p mode:=connect -p host:=192.168.1.50
    ros2 run zeus_rerun rerun_node --ros-args -p mode:=save -p path:=/home/pi/run.rrd

Parameters
    mode       connect | save | spawn
    host       viewer's IP, for connect
    port       viewer's port, default 9876
    path       .rrd file, for save
    decimate   log every Nth state; 10 = 100 Hz of plots (1 kHz / 10)

The viewer is never in the control path. A dropped Wi-Fi connection costs
plots, never a command.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from zeus_link.qos import COMMAND_QOS, STATE_QOS
from zeus_msgs.msg import NexusCommand, NexusState

from . import viz


class RerunNode(Node):

    def __init__(self) -> None:
        super().__init__("zeus_rerun")
        mode = str(self.declare_parameter("mode", "save").value)
        host = str(self.declare_parameter("host", "").value)
        port = int(self.declare_parameter("port", viz.VIEWER_PORT).value)
        path = str(self.declare_parameter("path", "zeus.rrd").value)
        self._decimate = max(1, int(self.declare_parameter("decimate", 10).value))

        where = viz.open_sink(mode, host=host, port=port, path=path)

        self._count = 0
        self._last_seq = None
        self.create_subscription(NexusState, "zeus/state", self._on_state, STATE_QOS)
        self.create_subscription(NexusCommand, "zeus/command", self._on_command, COMMAND_QOS)

        self.get_logger().info(
            f"{where}; logging every {self._decimate} state(s) "
            f"(~{1000 / self._decimate:.0f} Hz)")

    def _on_state(self, msg: NexusState) -> None:
        self._last_seq = msg.seq
        self._count += 1
        if self._count % self._decimate == 0:
            viz.log_state(msg)

    def _on_command(self, msg: NexusCommand) -> None:
        viz.log_residual(self._last_seq, msg.residual_rad)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RerunNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as exc:
        get_logger("zeus_rerun").fatal(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
