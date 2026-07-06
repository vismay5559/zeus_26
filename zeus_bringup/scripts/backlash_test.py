#!/usr/bin/env python3
"""
Actuator Backlash Measurement — ODrive S1 with 47:1 gearbox.

ODrive is configured with the AS5047P load encoder (after gearbox) as primary.
So Set_Input_Pos and Get_Encoder_Estimates both use OUTPUT turns (after gearbox).
No gear ratio conversion needed — 10° output = 10/360 output turns.

Does NOT require the ROS stack.
Run: python3 zeus_bringup/scripts/backlash_test.py
"""

import socket
import struct
import time
import statistics

# ── Configuration ─────────────────────────────────────────────────────────────
CAN_IFACE      = "can_odrive"
NODE_ID        = 0

N_CYCLES       = 250     # number of +/- approach pairs

# How far to overshoot the target before coming back.
# In OUTPUT degrees. Must be >> backlash gap (typically < 0.5° for good gearboxes).
# Keep small to avoid hitting joint limits.
APPROACH_DEG   = 10.0    # output degrees

# Speed of movement — output degrees per second.
MOVE_SPEED_DEG_S = 50.0  # output °/s
STEP_PERIOD_S    = 0.02  # 20 ms between steps → 50 Hz ramp

SETTLE_TIME_S  = 2.0     # seconds to wait after reaching target before reading position
OUTPUT_PLOT    = "/tmp/backlash_plot.png"
# ─────────────────────────────────────────────────────────────────────────────

# ODrive units are OUTPUT turns (AS5047P load encoder is primary).
# 1 output turn = 360° of the joint/output shaft.
# 10° output = 10/360 = 0.0278 output turns.
APPROACH_TURNS  = APPROACH_DEG / 360.0          # output turns to overshoot by
DEG_PER_STEP    = MOVE_SPEED_DEG_S * STEP_PERIOD_S
TURNS_PER_STEP  = DEG_PER_STEP / 360.0          # output turns per ramp step

_STATE_IDLE                = 1
_STATE_CLOSED_LOOP_CONTROL = 8
_CONTROL_MODE_POSITION     = 3
_INPUT_MODE_PASSTHROUGH    = 1
_CMD_SET_AXIS_STATE        = 0x007
_CMD_ENCODER_ESTIMATES     = 0x009
_CMD_SET_CONTROLLER_MODE   = 0x00B
_CMD_SET_INPUT_POS         = 0x00C


# ── CAN helpers ───────────────────────────────────────────────────────────────

def can_send(node_id: int, cmd: int, data: bytes) -> None:
    """Plain (non-FD) socket for sending classic CAN frames — same as commander.py."""
    can_id = (node_id << 5) | cmd
    payload = (data + b'\x00' * 8)[:8]
    frame = struct.pack('=IB3x8s', can_id, 8, payload)
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((CAN_IFACE,))
        sock.send(frame)
    finally:
        sock.close()


def open_recv_sock() -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
    sock.bind((CAN_IFACE,))
    sock.settimeout(0.1)
    return sock


def read_encoder(recv_sock, timeout: float = 1.0):
    """Return (pos_output_turns, vel_output_turns_s) or (None, None)."""
    target_id = (NODE_ID << 5) | _CMD_ENCODER_ESTIMATES
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            raw = recv_sock.recv(72)
        except socket.timeout:
            continue
        if len(raw) < 16:
            continue
        if (struct.unpack_from('=I', raw, 0)[0] & 0x7FF) == target_id:
            return struct.unpack_from('<f', raw, 8)[0], struct.unpack_from('<f', raw, 12)[0]
    return None, None


def move_to(recv_sock, target_output_turns: float) -> float:
    """
    Ramp from current position to target in small steps.

    Each step = TURNS_PER_STEP output turns, sent every STEP_PERIOD_S seconds.
    Effective speed = MOVE_SPEED_DEG_S output degrees/second.
    This avoids large position jumps that cause overvoltage from hard braking.

    Returns the settled output-turn position after SETTLE_TIME_S.
    """
    current, _ = read_encoder(recv_sock, timeout=1.0)
    if current is None:
        print("  ERROR: lost encoder signal")
        return None

    distance = target_output_turns - current
    n_steps = max(1, int(abs(distance) / TURNS_PER_STEP) + 1)

    for i in range(1, n_steps + 1):
        step_pos = current + (distance * i / n_steps)
        can_send(NODE_ID, _CMD_SET_INPUT_POS,
                 struct.pack('<fhh', step_pos, 0, 0))
        time.sleep(STEP_PERIOD_S)

    time.sleep(SETTLE_TIME_S)

    settled, _ = read_encoder(recv_sock, timeout=1.0)
    return settled


def to_deg(output_turns: float) -> float:
    return output_turns * 360.0


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  Actuator Backlash Test")
    print("=" * 60)
    print(f"  Interface    : {CAN_IFACE}   node_id = {NODE_ID}")
    print(f"  Encoder units: OUTPUT turns (AS5047P load encoder is primary)")
    print(f"  Cycles       : {N_CYCLES}")
    print(f"  Overshoot    : ±{APPROACH_DEG}° output = ±{APPROACH_TURNS:.5f} output turns")
    print(f"  Move speed   : {MOVE_SPEED_DEG_S}°/s output")
    print(f"  Settle time  : {SETTLE_TIME_S} s")
    print()

    recv_sock = open_recv_sock()

    print("[1/4] IDLE (clear errors) ...")
    can_send(NODE_ID, _CMD_SET_AXIS_STATE, struct.pack('<I', _STATE_IDLE) + b'\x00' * 4)
    time.sleep(0.4)

    print("[2/4] POSITION_CONTROL + PASSTHROUGH ...")
    can_send(NODE_ID, _CMD_SET_CONTROLLER_MODE,
             struct.pack('<II', _CONTROL_MODE_POSITION, _INPUT_MODE_PASSTHROUGH))
    time.sleep(0.1)

    print("[3/4] CLOSED_LOOP_CONTROL ...")
    can_send(NODE_ID, _CMD_SET_AXIS_STATE,
             struct.pack('<I', _STATE_CLOSED_LOOP_CONTROL) + b'\x00' * 4)
    time.sleep(0.6)

    print("[4/4] Reading home position ...")
    home, _ = read_encoder(recv_sock, timeout=2.0)
    if home is None:
        print("ERROR: no encoder frames — is can_odrive up?")
        recv_sock.close()
        return

    print(f"  HOME = {home:.5f} output turns = {to_deg(home):.2f}° output (absolute)")
    print(f"  Overshoot targets: "
          f"{to_deg(home + APPROACH_TURNS):.3f}°  and  "
          f"{to_deg(home - APPROACH_TURNS):.3f}° (absolute output)")
    print()
    print("  Verify both targets are within safe joint limits. Ctrl-C to abort (3s) ...")
    time.sleep(3)
    print()
    print(f"  {'cycle':>5}  {'from +side':>16}  {'from -side':>16}  {'backlash':>10}")
    print(f"  {'':->5}  {'(output turns)':>16}  {'(output turns)':>16}  {'(output °)':>10}")

    pos_positive = []
    pos_negative = []

    try:
        for i in range(N_CYCLES):
            move_to(recv_sock, home + APPROACH_TURNS)   # overshoot positive
            pos_a = move_to(recv_sock, home)             # back to home from above

            move_to(recv_sock, home - APPROACH_TURNS)   # overshoot negative
            pos_b = move_to(recv_sock, home)             # back to home from below

            if pos_a is None or pos_b is None:
                print(f"\n  cycle {i+1}: encoder timeout — stopping")
                break

            pos_positive.append(pos_a)
            pos_negative.append(pos_b)

            bl_deg = to_deg(abs(pos_a - pos_b))
            print(f"  {i+1:5d}  {pos_a:+16.6f}  {pos_b:+16.6f}  {bl_deg:10.5f}°")

    except KeyboardInterrupt:
        print("\n[stopped early]")

    print()
    print("Returning to home ...")
    move_to(recv_sock, home)
    can_send(NODE_ID, _CMD_SET_AXIS_STATE, struct.pack('<I', _STATE_IDLE) + b'\x00' * 4)
    recv_sock.close()

    if len(pos_positive) < 3:
        print("Not enough data.")
        return

    backlash_deg = [to_deg(abs(a - b)) for a, b in zip(pos_positive, pos_negative)]
    mean_bl  = statistics.mean(backlash_deg)
    stdev_bl = statistics.stdev(backlash_deg)

    print("=" * 60)
    print("  RESULTS")
    print("=" * 60)
    print(f"  Samples       : {len(backlash_deg)}")
    print(f"  Mean backlash : {mean_bl:.5f}°  ({mean_bl * 60:.3f} arc-min)")
    print(f"  Std deviation : {stdev_bl:.5f}°  ({stdev_bl * 60:.3f} arc-min)")
    print(f"  Min / Max     : {min(backlash_deg):.5f}° / {max(backlash_deg):.5f}°")
    print()

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        fig.suptitle(
            f'Actuator Backlash — ODrive S1  47:1 gearbox\n'
            f'Mean = {mean_bl:.5f}°  ({mean_bl*60:.2f} arcmin)  '
            f'std = {stdev_bl:.5f}°  |  n = {len(backlash_deg)}',
            fontsize=12)

        cycles = range(1, len(backlash_deg) + 1)
        ax1.plot(cycles, backlash_deg, color='steelblue', lw=0.9, alpha=0.8)
        ax1.axhline(mean_bl, color='crimson', ls='--', lw=1.5,
                    label=f'mean = {mean_bl:.5f}°')
        ax1.fill_between(cycles, mean_bl - stdev_bl, mean_bl + stdev_bl,
                         alpha=0.2, color='crimson', label=f'±1σ = {stdev_bl:.5f}°')
        ax1.set_xlabel('Cycle')
        ax1.set_ylabel('Backlash (output degrees)')
        ax1.set_title('Backlash per cycle')
        ax1.legend(fontsize=9)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(bottom=0)

        ax2.hist(backlash_deg, bins=min(20, len(backlash_deg) // 3 + 2),
                 color='steelblue', edgecolor='white', alpha=0.85)
        ax2.axvline(mean_bl, color='crimson', ls='--', lw=1.5,
                    label=f'mean = {mean_bl:.5f}°')
        ax2.axvline(mean_bl - stdev_bl, color='crimson', ls=':', lw=1)
        ax2.axvline(mean_bl + stdev_bl, color='crimson', ls=':', lw=1,
                    label=f'±1σ = {stdev_bl:.5f}°')
        ax2.set_xlabel('Backlash (output degrees)')
        ax2.set_ylabel('Count')
        ax2.set_title('Distribution')
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(OUTPUT_PLOT, dpi=150, bbox_inches='tight')
        print(f"Plot saved → {OUTPUT_PLOT}")
        print(f"Copy to VS Code: cp {OUTPUT_PLOT} ~/zeus_26/")

    except ImportError:
        print("matplotlib not installed.")


if __name__ == '__main__':
    main()
