#!/usr/bin/env python3
"""
Real-time trajectory recorder and plotter.

Records:
  - Target: /forward_command_controller/commands  (ROS topic, in ODrive turns)
  - Actual: ODrive encoder estimate (CAN frame 0x009, pos_estimate in turns)

Both are converted to degrees for the plot.

Because command_only_mode mirrors commands back as states, the /dynamic_joint_states
topic does NOT show the real motor position — only the raw CAN frames do.

Run in a separate terminal WHILE the launch and commander are running:

  ros2 run zeus_bringup trajectory_plotter.py          # 10 s default
  ros2 run zeus_bringup trajectory_plotter.py 20       # 20 s recording

Output: /tmp/trajectory_plot.png  (copy with: scp pi:/tmp/trajectory_plot.png .)
"""

import socket
import struct
import sys
import threading
import time

import matplotlib
matplotlib.use('Agg')          # no display needed — saves to file
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ── Configure ─────────────────────────────────────────────────────────────────
CAN_IFACE      = "can_odrive"
NODE_ID        = 0
CYCLE_PERIOD_S = 0.900
OUT_PATH       = "/tmp/trajectory_plot.png"
# ─────────────────────────────────────────────────────────────────────────────

_CMD_ENCODER_ESTIMATES = 0x009


class TrajectoryRecorder(Node):

    def __init__(self, duration: float):
        super().__init__('trajectory_plotter')

        self._duration   = duration
        self._t0         = None          # set on first received sample
        self._lock       = threading.Lock()
        self._targets    = []            # [(t_s, deg)]
        self._actuals    = []            # [(t_s, deg)]
        self._done       = False

        self.create_subscription(
            Float64MultiArray,
            '/forward_command_controller/commands',
            self._on_cmd,
            10,
        )

        # CAN reader runs in a daemon thread so rclpy.spin() can stay on main.
        t = threading.Thread(target=self._can_reader, daemon=True)
        t.start()

        # Stop timer
        self.create_timer(duration, self._finish)
        self.get_logger().info(
            f'Recording target + ODrive actual for {duration:.0f} s ...'
            f'  Output → {OUT_PATH}')

    def _on_cmd(self, msg: Float64MultiArray) -> None:
        if self._done:
            return
        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            self._targets.append((now - self._t0, msg.data[0] * 360.0))

    def _can_reader(self) -> None:
        target_id = (NODE_ID << 5) | _CMD_ENCODER_ESTIMATES
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
            sock.bind((CAN_IFACE,))
            sock.settimeout(0.1)
            while not self._done:
                try:
                    raw = sock.recv(72)
                except socket.timeout:
                    continue
                if len(raw) < 12:
                    continue
                now   = time.monotonic()
                can_id = struct.unpack_from('=I', raw, 0)[0] & 0x7FF
                if can_id == target_id:
                    pos_rev = struct.unpack_from('<f', raw, 8)[0]
                    with self._lock:
                        if self._t0 is None:
                            self._t0 = now
                        self._actuals.append((now - self._t0, pos_rev * 360.0))
        finally:
            sock.close()

    def _finish(self) -> None:
        self._done = True
        self.get_logger().info('Recording complete — generating plot ...')
        time.sleep(0.1)   # let CAN thread drain last frame

        with self._lock:
            targets = list(self._targets)
            actuals = list(self._actuals)

        if not targets and not actuals:
            self.get_logger().error(
                'No data recorded. Is the commander running? Is the CAN bus up?')
            rclpy.shutdown()
            return

        self._plot(targets, actuals)
        rclpy.shutdown()

    def _plot(self, targets, actuals) -> None:
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        fig.suptitle('Hip Pitch R — Target vs ODrive Actual', fontsize=13)

        ax_pos, ax_err = axes

        # ── Position overlay ──────────────────────────────────────────────────
        if targets:
            t_t, y_t = zip(*targets)
            ax_pos.plot(t_t, y_t, color='royalblue', linewidth=0.8,
                        label='Target (commanded)')
        if actuals:
            t_a, y_a = zip(*actuals)
            ax_pos.plot(t_a, y_a, color='tomato', linewidth=1.0,
                        label='Actual (ODrive encoder)')

        ax_pos.set_ylabel('Position (°)')
        ax_pos.legend(loc='upper right', fontsize=9)
        ax_pos.grid(True, alpha=0.3)

        # Gait cycle markers
        if targets:
            t_max = max(t for t, _ in targets)
            for k in range(int(t_max / CYCLE_PERIOD_S) + 2):
                ax_pos.axvline(k * CYCLE_PERIOD_S, color='gray',
                               linestyle='--', linewidth=0.4, alpha=0.5)

        # ── Tracking error (interpolate actuals onto target times) ────────────
        if targets and actuals:
            t_t  = np.array([x[0] for x in targets])
            y_t  = np.array([x[1] for x in targets])
            t_a  = np.array([x[0] for x in actuals])
            y_a  = np.array([x[1] for x in actuals])

            # Only interpolate over the time range covered by both streams
            t_start = max(t_t[0],  t_a[0])
            t_end   = min(t_t[-1], t_a[-1])
            mask    = (t_t >= t_start) & (t_t <= t_end)

            if mask.sum() > 1:
                y_actual_interp = np.interp(t_t[mask], t_a, y_a)
                error_deg       = y_t[mask] - y_actual_interp
                ax_err.plot(t_t[mask], error_deg, color='darkorange',
                            linewidth=0.6, label='Error = target − actual')
                ax_err.axhline(0, color='black', linewidth=0.5)
                rms = float(np.sqrt(np.mean(error_deg ** 2)))
                ax_err.set_title(f'Tracking error  (RMS = {rms:.2f}°)', fontsize=10)

        ax_err.set_ylabel('Error (°)')
        ax_err.set_xlabel('Time (s)')
        ax_err.legend(loc='upper right', fontsize=9)
        ax_err.grid(True, alpha=0.3)
        ax_err.xaxis.set_major_locator(ticker.MultipleLocator(0.9))

        fig.tight_layout()
        fig.savefig(OUT_PATH, dpi=150, bbox_inches='tight')
        self.get_logger().info(f'Plot saved → {OUT_PATH}')
        self.get_logger().info(f'Copy to your laptop:  scp vismay@<pi-ip>:{OUT_PATH} .')
        plt.close(fig)


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

    rclpy.init()
    node = TrajectoryRecorder(duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
