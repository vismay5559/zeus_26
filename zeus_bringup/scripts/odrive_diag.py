#!/usr/bin/env python3
"""
ODrive S1 diagnostic — reads the heartbeat and reports axis state + errors.
Run this in a separate terminal while can0 is up.

Usage: python3 odrive_diag.py
"""
import socket
import struct
import time

NODE_ID   = 0       # change if your ODrive has a different node_id
CAN_IFACE = "can_odrive"

AXIS_STATES = {
    0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE", 4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH", 7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL", 9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND",
}
AXIS_ERRORS = {
    0x00000001: "INVALID_STATE",     0x00000008: "MOTOR_FAILED",
    0x00000020: "ENCODER_FAILED",    0x00000040: "CONTROLLER_FAILED",
    0x00000400: "WATCHDOG_TIMER_EXPIRED",
    0x00010000: "OVER_TEMP",         0x00020000: "UNKNOWN_POSITION",
}

HEARTBEAT_ID = (NODE_ID << 5) | 0x001
ENCODER_ID   = (NODE_ID << 5) | 0x009
TORQUE_ID    = (NODE_ID << 5) | 0x01C

# Enable CAN-FD so we can receive both frame types
sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
try:
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FD_FRAMES, 1)
except Exception:
    pass
sock.bind((CAN_IFACE,))
sock.settimeout(3.0)

print(f"Listening on {CAN_IFACE} for ODrive node {NODE_ID} frames (3 s timeout)...")
print(f"Expected heartbeat CAN ID: 0x{HEARTBEAT_ID:03X}\n")

heartbeat_found = False
encoder_found   = False
deadline = time.time() + 5.0

while time.time() < deadline:
    try:
        raw = sock.recv(72)  # CANFD_MTU; also handles CAN_MTU (16 bytes padded)
    except socket.timeout:
        break

    if len(raw) < 8:
        continue
    can_id = struct.unpack_from('=I', raw, 0)[0] & 0x7FF

    if can_id == HEARTBEAT_ID and not heartbeat_found:
        heartbeat_found = True
        axis_error = struct.unpack_from('<I', raw, 8)[0]
        axis_state = raw[12]
        state_name = AXIS_STATES.get(axis_state, f"UNKNOWN({axis_state})")
        errors = [n for m, n in AXIS_ERRORS.items() if axis_error & m]

        print("=" * 55)
        print(f"  HEARTBEAT from ODrive node {NODE_ID}")
        print(f"  Axis state : {axis_state} = {state_name}")
        print(f"  Axis error : 0x{axis_error:08X}  {', '.join(errors) if errors else '(none)'}")
        print("=" * 55)

        if axis_state == 8:
            print("✓  ODrive is in CLOSED_LOOP_CONTROL")
            print("   If motor still doesn't move, check whether ODrive is")
            print("   configured for CAN-FD or classic CAN (see instructions below).")
        elif axis_state == 1:
            print("✗  ODrive is IDLE — position commands are ignored.")
            if axis_error:
                print("   Has errors. Fix: odrv0.clear_errors() in odrivetool")
            else:
                print("   May need calibration:")
                print("   odrv0.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE")
        else:
            print(f"✗  ODrive is in state {state_name} — not ready for position commands.")
        print()

    elif can_id == ENCODER_ID and not encoder_found:
        encoder_found = True
        pos = struct.unpack_from('<f', raw, 8)[0]
        vel = struct.unpack_from('<f', raw, 12)[0]
        print(f"  Encoder estimate : pos={pos:.4f} rev  vel={vel:.4f} rev/s")
        print(f"  ↑ ODrive IS broadcasting classic CAN encoder frames.")
        print(f"    Frame type = {'FD' if len(raw) == 72 else 'classic CAN'}\n")

if not heartbeat_found:
    print("✗  No heartbeat received. Check:")
    print("   1) ODrive powered on?")
    print(f"  2) CAN bus up?  →  ip link show {CAN_IFACE}")
    print(f"  3) Node ID correct? (currently set to {NODE_ID})")
    print(f"  4) Expected ID was 0x{HEARTBEAT_ID:03X}")

sock.close()
