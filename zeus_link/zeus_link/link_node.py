"""
The one process that talks to the STM32.

Only one program can hold the serial port, so everything else on the Pi - the
policy, rerun, a bag recording, `ros2 topic echo` - goes through this node.

    STM32 ──USB 1 kHz──► link_node ──► /zeus/state    zeus_msgs/NexusState
                                   ──► /joint_states  sensor_msgs/JointState
    STM32 ◄──USB 250 Hz── link_node ◄── /zeus/command zeus_msgs/NexusCommand

Services:
    /zeus/stand_down   std_srvs/Trigger   latch: forward every command with
                                          enable false, and send one now
    /zeus/resume       std_srvs/Trigger   release that latch

Parameters (zeus_bringup/config/link.yaml):
    port                 "auto" or a device path
    joint_state_rate_hz  /joint_states rate; 0 disables it
    stats_period_sec     how often to log link health
    frame_id             header.frame_id for published messages

Reading and publishing are decoupled. NexusLink's thread drains the port at
1 kHz no matter what; this node's publish thread takes the newest packet each
time it is ready. If publishing ever falls behind, packets are skipped - never
queued - so /zeus/state cannot go stale. The periodic log shows both rates.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from zeus_msgs.msg import NexusCommand, NexusState

from .convert import (RESIDUAL_LIMIT_RAD, fill_state_msg, joint_state_arrays,
                      names_of, residual_rad_to_turns)
from .nexus_link import NexusLink
from .nexus_proto import (HEALTH_NAMES, JOINT_NAMES, NUM_JOINTS, SAFETY_NAMES,
                          STREAM_LEG_TEST)
from .odrive_names import axis_state_name, error_names
from .ports import PortError, find_port
from .qos import COMMAND_QOS, STATE_QOS

FUSION_NAMES = {0: "INVALID", 1: "CONVERGING", 2: "OK"}

# No packet for this long after opening the port gets a specific warning. The
# port opening is not proof of a running robot: a board flashed with a test
# mode in nexus_mode.h enumerates the same USB device and never sends a packet.
FIRST_PACKET_WARN_SEC = 2.0


class LinkNode(Node):

    def __init__(self) -> None:
        super().__init__("zeus_link")

        requested = self.declare_parameter("port", "auto").value
        self._js_rate = float(self.declare_parameter("joint_state_rate_hz", 100.0).value)
        stats_period = float(self.declare_parameter("stats_period_sec", 5.0).value)
        self._frame_id = str(self.declare_parameter("frame_id", "base_link").value)

        self._port = find_port(requested)   # PortError propagates to main()

        self._state_pub = self.create_publisher(NexusState, "zeus/state", STATE_QOS)
        self._js_pub = (self.create_publisher(JointState, "joint_states", 10)
                        if self._js_rate > 0.0 else None)
        self.create_subscription(NexusCommand, "zeus/command", self._on_command, COMMAND_QOS)
        self.create_service(Trigger, "zeus/stand_down", self._on_stand_down)
        self.create_service(Trigger, "zeus/resume", self._on_resume)

        self._stood_down = False
        self._published = 0
        self._commands = 0
        self._last_rx_packets = 0
        self._last_published = 0
        self._last_commands = 0
        self._last_report = time.monotonic()
        self._opened_at = time.monotonic()

        # What was last reported, so the log carries changes rather than a
        # repeat of the same state 1000 times a second.
        self._seen_safety = None
        self._seen_source = None
        self._seen_fusion = None
        self._seen_health = None
        self._seen_act_error = [0] * NUM_JOINTS
        self._seen_act_state = [None] * NUM_JOINTS

        self._stop = threading.Event()
        self._new_packet = threading.Event()
        self._link = NexusLink(self._port, on_packet=lambda _pkt: self._new_packet.set())
        self._link.start()

        self._pub_thread = threading.Thread(target=self._publish_loop,
                                            name="zeus-publish", daemon=True)
        self._pub_thread.start()

        self.create_timer(stats_period, self._report)

        self.get_logger().info(
            f"link open on {self._port}"
            + ("" if requested == "auto" else " (set by the port parameter)")
            + f"; publishing zeus/state, joint_states at {self._js_rate:g} Hz")

    # ---- STM32 -> ROS -----------------------------------------------------

    def _publish_loop(self) -> None:
        msg = NexusState()
        msg.header.frame_id = self._frame_id
        js = JointState()
        js_period = (1.0 / self._js_rate) if self._js_rate > 0.0 else 0.0
        last_js = 0.0
        last_seq = None

        while not self._stop.is_set():
            if not self._new_packet.wait(timeout=0.5):
                continue
            self._new_packet.clear()

            pkt = self._link.latest()
            if pkt is None or pkt.seq == last_seq:
                continue
            last_seq = pkt.seq

            # Nothing may end this loop but _stop. It is the only thing
            # publishing /zeus/state, and a thread that dies here leaves the
            # policy acting on its last observation while commands still flow -
            # which is what happened, at the first FAULT, before this guard.
            try:
                stamp = self.get_clock().now().to_msg()
                msg.header.stamp = stamp
                fill_state_msg(msg, pkt)
                self._state_pub.publish(msg)
                self._published += 1

                now = time.monotonic()
                if self._js_pub is not None and (now - last_js) >= js_period:
                    last_js = now
                    js.header.stamp = stamp
                    js.name, js.position, js.velocity, js.effort = joint_state_arrays(pkt)
                    self._js_pub.publish(js)

                self._log_changes(pkt)
            except Exception as exc:    # noqa: BLE001 - logged, and the stream goes on
                if self._stop.is_set() or not rclpy.ok():
                    break               # shutting down: the context went first
                self.get_logger().error(f"state publish failed: {exc!r}",
                                        throttle_duration_sec=1.0)

    def _log_changes(self, pkt) -> None:
        log = self.get_logger()

        source = bool(pkt.stream_flags & STREAM_LEG_TEST)
        if source != self._seen_source:
            if source:
                log.warn("source: the STM32 single-leg bench test (NEXUS_MODE_LEG_CAN). "
                         "Only its joints are real, and the board ignores commands.")
            else:
                log.info("source: the STM32 robot loop (NEXUS_MODE_ROBOT)")
            self._seen_source = source

        # One severity per line of code. rclpy keys its logging state on the
        # call site and raises if the same line logs at two severities, so
        # "(log.warn if fault else log.info)(...)" throws on the first FAULT.
        if pkt.safety_state != self._seen_safety:
            name = SAFETY_NAMES.get(pkt.safety_state, str(pkt.safety_state))
            if name == "FAULT":
                log.warn(f"safety state -> {name}")
            else:
                log.info(f"safety state -> {name}")
            self._seen_safety = pkt.safety_state

        if pkt.fused_valid != self._seen_fusion:
            log.info(f"estimator -> {FUSION_NAMES.get(pkt.fused_valid, pkt.fused_valid)}")
            self._seen_fusion = pkt.fused_valid

        if pkt.health != self._seen_health:
            faults = [n for bit, n in HEALTH_NAMES.items() if pkt.health & bit]
            if faults:
                log.warn("health faults: " + ", ".join(faults))
            elif self._seen_health is not None:
                log.info("health: all watched subsystems OK")
            self._seen_health = pkt.health

        for j in range(NUM_JOINTS):
            err = pkt.act_error[j]
            if err != self._seen_act_error[j]:
                if err:
                    log.error(f"{JOINT_NAMES[j]}: ODrive error 0x{err:08X} "
                              + " | ".join(error_names(err)))
                else:
                    log.info(f"{JOINT_NAMES[j]}: ODrive error cleared")
                self._seen_act_error[j] = err

            st = pkt.act_state[j]
            if st != self._seen_act_state[j]:
                if self._seen_act_state[j] is not None:
                    log.info(f"{JOINT_NAMES[j]}: axis {axis_state_name(st)}")
                self._seen_act_state[j] = st

    # ---- ROS -> STM32 -----------------------------------------------------

    def _on_command(self, msg: NexusCommand) -> None:
        try:
            turns, over = residual_rad_to_turns(msg.residual_rad)
        except ValueError as exc:
            self.get_logger().error(f"command dropped: {exc}", throttle_duration_sec=1.0)
            return

        if over:
            self.get_logger().warn(
                "residual beyond the STM32's +/-%.4f rad for %s - the board will "
                "reject this command" % (RESIDUAL_LIMIT_RAD, ", ".join(names_of(over))),
                throttle_duration_sec=1.0)

        self._send(turns, bool(msg.enable) and not self._stood_down)
        self._commands += 1

    def _send(self, turns, enable: bool) -> None:
        try:
            self._link.send_command(turns, enable=enable)
        except Exception as exc:    # serial write on an unplugged board, mostly
            self.get_logger().error(f"USB write failed: {exc}", throttle_duration_sec=1.0)

    def _on_stand_down(self, _req, resp):
        self._stood_down = True
        self._send([0.0] * NUM_JOINTS, enable=False)
        resp.success = True
        resp.message = ("standing down: commands are forwarded with enable false "
                        "until zeus/resume")
        self.get_logger().warn("stand down requested")
        return resp

    def _on_resume(self, _req, resp):
        self._stood_down = False
        resp.success = True
        resp.message = "enable is passed through again"
        self.get_logger().info("resume requested")
        return resp

    # ---- health of the link itself ----------------------------------------

    def _report(self) -> None:
        now = time.monotonic()
        dt = max(now - self._last_report, 1e-6)
        stats = self._link.stats

        rx_hz = (stats.packets - self._last_rx_packets) / dt
        pub_hz = (self._published - self._last_published) / dt
        cmd_hz = (self._commands - self._last_commands) / dt
        self._last_rx_packets = stats.packets
        self._last_published = self._published
        self._last_commands = self._commands
        self._last_report = now

        if stats.packets == 0:
            if now - self._opened_at > FIRST_PACKET_WARN_SEC:
                self.get_logger().warn(
                    f"no packets from {self._port} yet. If the board is powered, check "
                    "NEXUS_MODE in Appli/App/nexus_mode.h is NEXUS_MODE_ROBOT - the test "
                    "modes enumerate the same USB device and never send a state packet.")
            return

        self.get_logger().info(
            f"rx {rx_hz:6.1f} Hz | published {pub_hz:6.1f} Hz | commands {cmd_hz:5.1f} Hz | "
            f"{stats}" + (" | STOOD DOWN" if self._stood_down else ""))

    def destroy_node(self) -> None:
        # Hand the actuators back before letting go of the port, so a stopped
        # node idles the robot now instead of 200 ms later via a link fault.
        try:
            for _ in range(3):
                self._send([0.0] * NUM_JOINTS, enable=False)
                time.sleep(0.005)
        finally:
            self._stop.set()
            self._pub_thread.join(timeout=1.0)
            self._link.stop()
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LinkNode()
        rclpy.spin(node)
    except PortError as exc:
        get_logger("zeus_link").fatal(str(exc))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
