"""
The ROS message definitions agree with the packet, field for field.

Three descriptions of the same data exist - NexusState.msg, convert.STATE_FIELDS
and the NexusState dataclass the parser fills - and nothing but this test keeps
them together.
"""

import dataclasses
import struct

from conftest import read_msg
from zeus_link import convert, nexus_proto as P

_PY_TYPE = {"f": "float32", "I": "uint32", "H": "uint16", "B": "uint8"}


def test_msg_fields_are_exactly_state_fields():
    fields, _ = read_msg("NexusState")
    body = [(t, c, n) for t, c, n in fields if n != "header"]
    assert [(n, t, c) for t, c, n in body] == [(n, t, c) for n, t, c in convert.STATE_FIELDS]
    assert fields[0] == ("std_msgs/Header", 1, "header")


def test_state_fields_cover_the_dataclass():
    # reserved0 is packet padding, deliberately not published.
    dc = [f.name for f in dataclasses.fields(P.NexusState) if f.name != "reserved0"]
    assert sorted(dc) == sorted(n for n, _, _ in convert.STATE_FIELDS)


def test_state_field_types_match_the_wire_format():
    # Walk STATE_FORMAT once, recording each named field's struct code and count.
    order = [
        "sync", "msg_id", "version", "seq", "timestamp_us", "pelvis_z", "quat", "gyro",
        "vel_hdg", "joint_pos", "joint_vel", "spring_angle", "ref_angle", "contact",
        "foot_z", "phase", "imu_quat", "imu_accel", "imu_gyro", "imu_seq", "act_torque",
        "act_error", "fused_pos", "fused_vel", "fused_gyro_bias", "fused_accel_bias",
        "overruns", "usb_dropped", "can_dropped", "loop_us_max", "enc_stalls",
        "can_bus_off", "stream_flags", "reserved0", "contact_ticks", "act_state",
        "act_flags", "enc_valid", "contacts", "fused_valid", "health", "fk_valid",
        "safety_state", "crc",
    ]
    import re
    chunks = re.findall(r"(\d*)([fIHB])", P.STATE_FORMAT)
    assert len(chunks) == len(order)
    wire = {name: (_PY_TYPE[code], int(n or 1)) for name, (n, code) in zip(order, chunks)}
    for name, typ, count in convert.STATE_FIELDS:
        assert wire[name] == (typ, count), name
    assert struct.calcsize(P.STATE_FORMAT) == 444


def test_joint_constants_match_joint_names():
    _, consts = read_msg("NexusState")
    for i, name in enumerate(P.JOINT_NAMES):
        assert consts["J_" + name.upper()] == i


def test_other_constants_match_the_protocol():
    _, c = read_msg("NexusState")
    assert (c["SAFETY_BOOT"], c["SAFETY_IDLE"], c["SAFETY_ARMED"], c["SAFETY_FAULT"]) == \
           (P.SAFETY_BOOT, P.SAFETY_IDLE, P.SAFETY_ARMED, P.SAFETY_FAULT)
    assert (c["FUSION_INVALID"], c["FUSION_CONVERGING"], c["FUSION_OK"]) == \
           (P.FUSION_INVALID, P.FUSION_CONVERGING, P.FUSION_OK)
    for bit in ("IMU", "ENC", "CAN1", "CAN2", "LINK", "TIMING"):
        assert c["HEALTH_" + bit] == getattr(P, "HEALTH_" + bit)
    assert c["CONTACTS_LEFT_FOOT_BIT"] == P.CONTACT_L_FOOT
    assert c["CONTACTS_RIGHT_FOOT_BIT"] == P.CONTACT_R_FOOT
    assert c["STREAM_GAIT_LIVE"] == P.STREAM_GAIT_LIVE
    assert (c["FK_RIGHT_VALID"], c["FK_LEFT_VALID"]) == (P.FK_RIGHT_VALID, P.FK_LEFT_VALID)
    assert [c["CONTACT_" + n.upper()] for n in P.CONTACT_NAMES] == [0, 1, 2, 3]


def test_command_msg():
    fields, _ = read_msg("NexusCommand")
    assert fields == [("std_msgs/Header", 1, "header"),
                      ("float32", 10, "residual_rad"),
                      ("bool", 1, "enable")]


def test_policy_fields_are_in_the_msg_in_order():
    names = [n for _, _, n in read_msg("NexusState")[0]]
    policy = [n for n, _ in P.POLICY_FIELDS]
    idx = [names.index(n) for n in policy]
    assert idx == sorted(idx) and idx == list(range(idx[0], idx[0] + len(idx)))
