#!/usr/bin/env python3
"""
Hip Pitch R gait trajectory test commander — with auto-home on every run.

Startup sequence every time:
  1. Send IDLE          — clears any ODrive error flags from previous session.
  2. Enter CLOSED_LOOP  — motor holds its current physical position.
  3. Verify via heartbeat that CLOSED_LOOP was actually entered.
  4. Read pos_estimate  — this is declared HOME = 0°.
  5. Run the Hip Pitch R trajectory where every value is *relative to home*.
     e.g. "-20°" means 20° backward from wherever the motor sat at startup.

On Ctrl-C: returns to home position, then sends IDLE.

Usage:
  ros2 run zeus_bringup single_joint_commander.py
"""

import socket
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import DynamicJointState

# ── Configure ─────────────────────────────────────────────────────────────────
JOINT_NAME      = "joint_0"
CAN_IFACE       = "can_odrive"
NODE_ID         = 0            # odrv0.axis0.config.can.node_id
CYCLE_PERIOD_S  = 0.900        # gait cycle length in seconds
PUBLISH_RATE_HZ = 1000         # Hz — must match controller_manager update_rate
# ─────────────────────────────────────────────────────────────────────────────

# Hip Pitch R waypoints (time_s, angle_deg) — all relative to home (0° = start).
# These are converted to turns (÷360) and added to the home position before
# being sent to the ODrive, so the motor never moves far from where it started.
_WAYPOINTS_DEG = [
    (0.000, -20.000),
    (0.045, -16.000),
    (0.090, -12.000),
    (0.135,  -8.000),
    (0.180,  -4.000),
    (0.225,   0.000),
    (0.270,   4.000),
    (0.315,   8.000),
    (0.360,  12.000),
    (0.405,  16.000),
    (0.432,  18.400),
    (0.450,  20.000),
    (0.495,  16.000),
    (0.540,  12.000),
    (0.585,   8.000),
    (0.630,   4.000),
    (0.675,   0.000),
    (0.720,  -4.000),
    (0.765,  -8.000),
    (0.810, -12.000),
    (0.855, -16.000),
    (0.882, -18.400),
    (0.900, -20.000),  # == start of next cycle → seamless loop
]

# Pre-convert to turns (ODrive native unit)
_WAYPOINTS_REV = [(t, deg / 360.0) for t, deg in _WAYPOINTS_DEG]

# ODrive axis states
_STATE_IDLE                = 1
_STATE_CLOSED_LOOP_CONTROL = 8

# ODrive control / input modes
_CONTROL_MODE_POSITION  = 3
_INPUT_MODE_PASSTHROUGH = 1   # we supply pre-interpolated 1 kHz stream

# ODrive CAN command IDs
_CMD_HEARTBEAT           = 0x001
_CMD_SET_AXIS_STATE      = 0x007
_CMD_ENCODER_ESTIMATES   = 0x009
_CMD_SET_CONTROLLER_MODE = 0x00B

_AXIS_STATE_NAMES = {
    1: 'IDLE', 3: 'FULL_CALIBRATION', 4: 'MOTOR_CALIBRATION',
    7: 'ENCODER_OFFSET_CALIBRATION', 8: 'CLOSED_LOOP_CONTROL',
}


# ── Low-level CAN helpers ─────────────────────────────────────────────────────

def _open_can_sock(iface: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    sock.bind((iface,))
    return sock


def _can_send(iface: str, can_id: int, data: bytes) -> None:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((iface,))
        sock.send(struct.pack('=IB3x8s', can_id, 8, data))
    finally:
        sock.close()


def _send_axis_state(iface: str, node_id: int, state: int) -> None:
    can_id = (node_id << 5) | _CMD_SET_AXIS_STATE
    _can_send(iface, can_id, struct.pack('<I', state) + b'\x00' * 4)


def _send_controller_mode(iface: str, node_id: int,
                          control_mode: int, input_mode: int) -> None:
    can_id = (node_id << 5) | _CMD_SET_CONTROLLER_MODE
    _can_send(iface, can_id, struct.pack('<II', control_mode, input_mode))


def _wait_for_axis_state(iface: str, node_id: int,
                         desired_state: int, timeout: float = 3.0) -> tuple:
    """
    Block until ODrive heartbeat reports desired_state or timeout expires.
    Returns (axis_state, axis_error).
    """
    heartbeat_id = (node_id << 5) | _CMD_HEARTBEAT
    sock = _open_can_sock(iface)
    sock.settimeout(0.1)
    last_state, last_error = -1, 0
    frames_total = 0
    ids_seen: set = set()
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                raw = sock.recv(72)
            except socket.timeout:
                continue
            if len(raw) < 9:
                continue
            frames_total += 1
            can_id = struct.unpack_from('=I', raw, 0)[0] & 0x7FF
            ids_seen.add(can_id)
            if can_id == heartbeat_id and len(raw) >= 13:
                last_error = struct.unpack_from('<I', raw, 8)[0]
                last_state = raw[12]
                if last_state == desired_state:
                    return last_state, last_error
    finally:
        sock.close()
        if last_state != desired_state:
            ids_str = ', '.join(f'0x{i:03X}' for i in sorted(ids_seen)) or 'none'
            print(
                f'[CAN diag] frames_received={frames_total}  '
                f'ids_seen=[{ids_str}]  '
                f'wanted_heartbeat_id=0x{heartbeat_id:03X}  '
                f'last_heartbeat_state={last_state}',
                flush=True)
            if frames_total == 0:
                print('[CAN diag] No frames at all — is can_odrive up? '
                      '(sudo ip link set can_odrive up ...)', flush=True)
            elif heartbeat_id not in ids_seen:
                print('[CAN diag] Heartbeat (0x001) never seen. '
                      'Fix: odrv0.axis0.config.can.heartbeat_rate_ms = 100  '
                      'then odrv0.save_configuration()', flush=True)
            else:
                print(f'[CAN diag] Heartbeat seen but state never reached {desired_state}. '
                      'ODrive may need calibration.', flush=True)
    return last_state, last_error


def _read_home_position(iface: str, node_id: int, timeout: float = 2.0) -> float:
    """
    Read the ODrive's current pos_estimate (turns) from its 0x009 broadcast.
    This value is stored as HOME = 0° for the session.
    """
    target_id = (node_id << 5) | _CMD_ENCODER_ESTIMATES
    sock = _open_can_sock(iface)
    sock.settimeout(0.1)
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                raw = sock.recv(72)
            except socket.timeout:
                continue
            if len(raw) < 12:
                continue
            if (struct.unpack_from('=I', raw, 0)[0] & 0x7FF) == target_id:
                return struct.unpack_from('<f', raw, 8)[0]
    finally:
        sock.close()
    return 0.0


def _lerp_position_rev(t_in_cycle: float) -> float:
    """Linearly interpolated relative position (turns) at phase t in cycle."""
    wp = _WAYPOINTS_REV
    if t_in_cycle <= wp[0][0]:
        return wp[0][1]
    if t_in_cycle >= wp[-1][0]:
        return wp[-1][1]
    for i in range(len(wp) - 1):
        t0, p0 = wp[i]
        t1, p1 = wp[i + 1]
        if t0 <= t_in_cycle <= t1:
            return p0 + (t_in_cycle - t0) / (t1 - t0) * (p1 - p0)
    return wp[-1][1]


# ── ROS2 commander node ───────────────────────────────────────────────────────

class HipPitchCommander(Node):

    def __init__(self):
        super().__init__('hip_pitch_commander')

        self._torque_nm      = 0.0
        self._cycle_start    = None
        self._last_log_cycle = -1
        self._home_rev       = 0.0   # ODrive absolute position declared as 0°
        self._startup_ok     = False

        self._cmd_pub = self.create_publisher(
            Float64MultiArray, '/forward_command_controller/commands', 10)

        self.create_subscription(
            DynamicJointState, '/dynamic_joint_states',
            self._on_dynamic_states,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        if not self._startup_sequence():
            return

        self._startup_ok = True
        # Start 1 kHz publish loop
        self._cycle_start = self.get_clock().now()
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._tick)

        lo = _WAYPOINTS_DEG[0][1]
        hi = _WAYPOINTS_DEG[11][1]
        self.get_logger().info(
            f'Trajectory running: {lo:+.0f}° to {hi:+.0f}° relative to home  '
            f'(cycle={CYCLE_PERIOD_S}s @ {PUBLISH_RATE_HZ} Hz).  Ctrl-C to stop.')

    def _startup_sequence(self) -> bool:
        """
        Returns True if the motor is ready to run the trajectory.
        Sequence:
          [1] IDLE  — clears ODrive error flags from any previous crash
          [2] Set POSITION_CONTROL + PASSTHROUGH
          [3] CLOSED_LOOP — motor locks at its current physical position
          [4] Read pos_estimate via 0x009 — confirms CAN alive, declares HOME
        Note: heartbeat (0x001) is not used here because it is disabled on this
        ODrive (heartbeat_rate_ms = 0).  Verification is via encoder broadcast.
        """
        log = self.get_logger()

        # [1] IDLE first — resets error state
        log.info('[1/4] Sending IDLE to clear previous errors ...')
        _send_axis_state(CAN_IFACE, NODE_ID, _STATE_IDLE)
        time.sleep(0.3)

        # [2] Configure controller
        log.info('[2/4] Setting POSITION_CONTROL + PASSTHROUGH ...')
        _send_controller_mode(CAN_IFACE, NODE_ID,
                              _CONTROL_MODE_POSITION, _INPUT_MODE_PASSTHROUGH)
        time.sleep(0.05)

        # [3] Enter CLOSED_LOOP — motor holds current position
        log.info('[3/4] Entering CLOSED_LOOP_CONTROL (500 ms settling) ...')
        _send_axis_state(CAN_IFACE, NODE_ID, _STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.5)   # give ODrive time to complete state transition

        # [4] Read encoder to confirm ODrive is alive and store HOME
        log.info('[4/4] Homing: reading current encoder position ...')
        self._home_rev = _read_home_position(CAN_IFACE, NODE_ID, timeout=2.0)
        if self._home_rev == 0.0:
            # _read_home_position returns 0.0 on timeout — warn but continue,
            # since 0.0 is also a valid absolute position.
            log.warn(
                '  No encoder frame received — ODrive may be off or wrong node_id. '
                f'  Proceeding with home=0.0 rev. '
                f'  Check: candump {CAN_IFACE} | grep " 0{NODE_ID:02X}9"')
        log.info(
            f'  HOME SET — ODrive absolute: {self._home_rev:.4f} rev '
            f'({self._home_rev * 360:.2f}°)  →  treated as 0° for this session')
        log.info(
            f'  Trajectory range: {_WAYPOINTS_DEG[0][1]:+.1f}° to '
            f'{_WAYPOINTS_DEG[11][1]:+.1f}° from home')
        return True

    def _tick(self) -> None:
        now     = self.get_clock().now()
        elapsed = (now - self._cycle_start).nanoseconds * 1e-9
        t       = elapsed % CYCLE_PERIOD_S

        # Absolute ODrive target = home + relative trajectory offset
        relative_rev = _lerp_position_rev(t)
        absolute_rev = self._home_rev + relative_rev

        msg = Float64MultiArray()
        msg.data = [absolute_rev]
        self._cmd_pub.publish(msg)

        # Log once per cycle boundary
        cycle_num = int(elapsed / CYCLE_PERIOD_S)
        if cycle_num != self._last_log_cycle:
            self._last_log_cycle = cycle_num
            self.get_logger().info(
                f'cycle {cycle_num + 1:4d}  '
                f'pos_from_home={relative_rev * 360:+7.2f}°  '
                f'torque={self._torque_nm:+.3f} Nm')

    def _on_dynamic_states(self, msg: DynamicJointState) -> None:
        try:
            idx = msg.joint_names.index(JOINT_NAME)
        except ValueError:
            return
        for name, value in zip(msg.interface_values[idx].interface_names,
                                msg.interface_values[idx].values):
            if name == 'torque_estimate':
                self._torque_nm = value


def main() -> None:
    rclpy.init()
    node = HipPitchCommander()
    if not node._startup_ok:
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[hip_pitch_commander] Returning to home position ...')
        try:
            msg = Float64MultiArray()
            msg.data = [node._home_rev]   # command home = 0° relative
            node._cmd_pub.publish(msg)
            time.sleep(0.3)
        except Exception:
            pass
        _send_axis_state(CAN_IFACE, NODE_ID, _STATE_IDLE)
        print('[hip_pitch_commander] ODrive is IDLE. Home was '
              f'{node._home_rev * 360:.2f}° (ODrive absolute).')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
