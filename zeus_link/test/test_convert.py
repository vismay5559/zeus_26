"""Packet -> message, command units, and the observation block."""

import math

import numpy as np
import pytest

from conftest import StrictMsg
from zeus_link import convert, nexus_proto as P


def test_fill_state_msg_sets_every_field_with_rclpy_types(packet):
    msg = StrictMsg("NexusState")
    convert.fill_state_msg(msg, packet)          # StrictMsg raises on any type slip
    for name, _, _ in convert.STATE_FIELDS:
        assert hasattr(msg, name), name


def test_fill_state_msg_preserves_values_including_nan(packet):
    msg = StrictMsg("NexusState")
    convert.fill_state_msg(msg, packet)
    assert msg.seq == 1234
    assert msg.act_error[9] == 0x08000200
    assert msg.act_state.dtype == np.uint8 and list(msg.act_state[:3]) == [8, 8, 1]
    assert math.isnan(float(msg.foot_z[1]))      # an invalid foot stays NaN, never 0.0
    np.testing.assert_allclose(msg.ref_angle, packet.ref_angle, rtol=1e-6)


def test_fill_state_msg_on_the_real_message_class_when_ros_is_present(packet):
    # Skipped off the robot. Under `colcon test` this runs against the classes
    # rosidl actually generated, which is the check StrictMsg stands in for.
    zeus_msgs = pytest.importorskip("zeus_msgs.msg")
    msg = zeus_msgs.NexusState()
    convert.fill_state_msg(msg, packet)
    assert msg.seq == packet.seq
    np.testing.assert_allclose(convert.policy_block(msg), convert.policy_block(packet))


def test_residual_is_converted_from_radians_to_turns():
    turns, over = convert.residual_rad_to_turns([2 * math.pi * 0.05] + [0.0] * 9)
    assert turns[0] == pytest.approx(0.05)
    assert over == []


def test_residual_over_the_board_limit_is_reported_not_clipped():
    rad = [0.0] * 10
    rad[P.JOINT_INDEX["right_knee_pitch"]] = -0.7          # 0.111 turns > 0.1
    turns, over = convert.residual_rad_to_turns(rad)
    assert over == [P.JOINT_INDEX["right_knee_pitch"]]
    assert turns[7] == pytest.approx(-0.7 / (2 * math.pi))
    assert convert.names_of(over) == ["right_knee_pitch"]


@pytest.mark.parametrize("bad", [[0.0] * 9, [0.0] * 11, [float("nan")] + [0.0] * 9,
                                 [0.0] * 9 + [float("inf")]])
def test_residual_shape_and_finiteness_are_enforced(bad):
    with pytest.raises(ValueError):
        convert.residual_rad_to_turns(bad)


def test_policy_block_matches_the_raw_bytes(packet, packet_bytes):
    from_packet = convert.policy_block(packet)
    raw = np.frombuffer(packet_bytes, dtype="<f4", count=P.POLICY_COUNT,
                        offset=P.POLICY_OFFSET)
    assert from_packet.dtype == np.float32 and from_packet.size == 52
    np.testing.assert_array_equal(np.isnan(from_packet), np.isnan(raw))
    np.testing.assert_array_equal(from_packet[~np.isnan(raw)], raw[~np.isnan(raw)])


def test_policy_block_is_identical_from_msg_and_packet(packet):
    msg = StrictMsg("NexusState")
    convert.fill_state_msg(msg, packet)
    a, b = convert.policy_block(msg), convert.policy_block(packet)
    np.testing.assert_array_equal(np.nan_to_num(a, nan=-1), np.nan_to_num(b, nan=-1))


def test_joint_state_arrays(packet):
    names, pos, vel, eff = convert.joint_state_arrays(packet)
    assert names == list(P.JOINT_NAMES)
    assert all(type(x) is float for x in pos + vel + eff)
    assert pos[4] == pytest.approx(0.5) and eff[9] == pytest.approx(4.5)
