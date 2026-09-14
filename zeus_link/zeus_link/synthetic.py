"""
Synthetic state packets, byte-exact to what the STM32 sends.

For tests, and for running a policy or a viewer with no robot attached:

    from zeus_link.synthetic import state_packet
    from zeus_link.nexus_proto import NexusState
    pkt = NexusState.parse(state_packet(seq=1))

Every value is distinct, so a field read from the wrong place shows up.
"""

import struct

from . import nexus_proto as P


def state_values(seq=1234, **over):
    """Field values in packet order, every one distinct so a swap is visible."""
    v = {
        "seq": seq,
        "timestamp_us": 987654321,
        "pelvis_z": 0.81,
        "quat": [1.0, 0.01, -0.02, 0.03],
        "gyro": [0.1, -0.2, 0.3],
        "vel_hdg": [0.05, 0.4, -0.01],
        "joint_pos": [0.1 * (i + 1) for i in range(10)],
        "joint_vel": [-0.01 * (i + 1) for i in range(10)],
        "spring_angle": [0.001, -0.002, 0.003, -0.004],
        "ref_angle": [0.2 * (i + 1) for i in range(10)],
        "contact": [1.0, 0.0, 0.0, 1.0],
        "foot_z": [0.0, float("nan")],
        "phase": 0.37,
        "imu_quat": [0.99, 0.0, 0.1, 0.0],
        "imu_accel": [0.0, 0.0, 9.81],
        "imu_gyro": [0.01, 0.02, 0.03],
        "imu_seq": 42,
        "act_torque": [0.5 * i for i in range(10)],
        "act_error": [0, 0, 0x200, 0, 0, 0, 0, 0, 0, 0x08000200],
        "fused_pos": [1.0, 2.0, 0.81],
        "fused_vel": [0.3, 0.1, 0.0],
        "fused_gyro_bias": [1e-3, 2e-3, 3e-3],
        "fused_accel_bias": [0.01, 0.02, 0.03],
        "overruns": 3,
        "usb_dropped": 7,
        "can_dropped": [11, 12],
        "loop_us_max": 245,
        "enc_stalls": 2,
        "can_bus_off": [0, 1],
        "stream_flags": P.STREAM_GAIT_LIVE,
        "reserved0": 0,
        "contact_ticks": [500, 65535],
        "act_state": [8, 8, 1, 8, 8, 8, 8, 8, 8, 3],
        "act_flags": [3, 3, 1, 3, 3, 3, 3, 3, 3, 0],
        "enc_valid": 0x0F,
        "contacts": 0x19,
        "fused_valid": P.FUSION_CONVERGING,
        "health": P.HEALTH_LINK,
        "fk_valid": P.FK_RIGHT_VALID,
        "safety_state": P.SAFETY_IDLE,
    }
    v.update(over)
    return v


def pack_state(values):
    flat = [P.SYNC, P.MSG_STATE, P.PROTO_VERSION]
    for name in (
        "seq", "timestamp_us", "pelvis_z", "quat", "gyro", "vel_hdg", "joint_pos",
        "joint_vel", "spring_angle", "ref_angle", "contact", "foot_z", "phase",
        "imu_quat", "imu_accel", "imu_gyro", "imu_seq", "act_torque", "act_error",
        "fused_pos", "fused_vel", "fused_gyro_bias", "fused_accel_bias", "overruns",
        "usb_dropped", "can_dropped", "loop_us_max", "enc_stalls", "can_bus_off",
        "stream_flags", "reserved0", "contact_ticks", "act_state", "act_flags",
        "enc_valid", "contacts", "fused_valid", "health", "fk_valid", "safety_state",
    ):
        x = values[name]
        flat.extend(x if isinstance(x, list) else [x])
    body = struct.pack(P.STATE_FORMAT[:-1], *flat)
    return body + struct.pack("<H", P.crc16(body))



def state_packet(seq=1234, **over):
    """A complete, CRC-valid 444-byte state packet."""
    return pack_state(state_values(seq=seq, **over))
