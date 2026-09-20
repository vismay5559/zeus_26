"""
Packet <-> ROS message conversion, with no ROS imports.

Kept free of rclpy on purpose: everything here is testable on any machine, and
the node that uses it stays a thin shell around these functions.

Three things live here:

  fill_state_msg(msg, pkt)      NexusState packet -> zeus_msgs/NexusState
  residual_rad_to_turns(values) the one unit conversion on the command path
  policy_block(state)           the 46-value observation, from a packet OR a msg
"""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from .nexus_proto import BOLTED_JOINT_NAMES, JOINT_NAMES, NUM_JOINTS, POLICY_FIELDS

TWO_PI = 2.0 * math.pi

# The STM32 rejects any residual larger than this (SAFETY_MAX_RESIDUAL_TURNS
# in safety.h). Mirrored here only to WARN - the board stays the authority.
RESIDUAL_LIMIT_TURNS = 0.1
RESIDUAL_LIMIT_RAD = RESIDUAL_LIMIT_TURNS * TWO_PI

# --------------------------------------------------------------------------
# Every field of zeus_msgs/NexusState except the header, as
# (name, ROS type, count). count 1 is a scalar, anything else a fixed array.
#
# test_msg_matches_proto.py checks this list three ways: against the .msg file,
# against the NexusState dataclass, and against the packet format - so a field
# added to one and forgotten in another fails a test instead of a robot.
# --------------------------------------------------------------------------
STATE_FIELDS: Tuple[Tuple[str, str, int], ...] = (
    ("seq", "uint32", 1),
    ("timestamp_us", "uint32", 1),
    ("pelvis_z", "float32", 1),
    ("quat", "float32", 4),
    ("gyro", "float32", 3),
    ("vel_hdg", "float32", 3),
    ("joint_pos", "float32", NUM_JOINTS),
    ("joint_vel", "float32", NUM_JOINTS),
    ("spring_angle", "float32", 4),
    ("ref_angle", "float32", NUM_JOINTS),
    ("contact", "float32", 4),
    ("foot_z", "float32", 2),
    ("phase", "float32", 1),
    ("imu_quat", "float32", 4),
    ("imu_accel", "float32", 3),
    ("imu_gyro", "float32", 3),
    ("imu_seq", "uint32", 1),
    ("act_torque", "float32", NUM_JOINTS),
    ("act_error", "uint32", NUM_JOINTS),
    ("act_state", "uint8", NUM_JOINTS),
    ("act_flags", "uint8", NUM_JOINTS),
    ("fused_pos", "float32", 3),
    ("fused_vel", "float32", 3),
    ("fused_gyro_bias", "float32", 3),
    ("fused_accel_bias", "float32", 3),
    ("overruns", "uint32", 1),
    ("usb_dropped", "uint32", 1),
    ("can_dropped", "uint16", 2),
    ("loop_us_max", "uint16", 1),
    ("enc_stalls", "uint16", 1),
    ("can_bus_off", "uint8", 2),
    ("stream_flags", "uint8", 1),
    ("contact_ticks", "uint16", 2),
    ("enc_valid", "uint8", 1),
    ("contacts", "uint8", 1),
    ("fused_valid", "uint8", 1),
    ("health", "uint8", 1),
    ("fk_valid", "uint8", 1),
    ("safety_state", "uint8", 1),
)

_NUMPY = {
    "float32": np.float32,
    "uint32": np.uint32,
    "uint16": np.uint16,
    "uint8": np.uint8,
}


def fill_state_msg(msg, pkt) -> None:
    """
    Copy a parsed packet into a zeus_msgs/NexusState, in place.

    Types are matched exactly, because the generated message classes assert on
    them: a float32 scalar must be a Python float, an integer scalar a Python
    int, and a fixed array is handed over as a numpy array of the exact dtype.
    The array form is also the fast path - the setter checks dtype and size
    once instead of type-checking every element, which matters at 1 kHz on a
    Raspberry Pi.

    The header is left to the caller, which knows the receive time.
    """
    for name, ros_type, count in STATE_FIELDS:
        value = getattr(pkt, name)
        if count == 1:
            setattr(msg, name, float(value) if ros_type == "float32" else int(value))
        else:
            setattr(msg, name, np.asarray(value, dtype=_NUMPY[ros_type]))


def residual_rad_to_turns(values: Sequence[float]) -> Tuple[List[float], List[int]]:
    """
    Convert a residual from radians to the turns the STM32 takes.

    Returns (turns, over) where `over` lists the joint indices whose residual
    exceeds what the board will accept. Those are still returned, not clipped:
    silently clipping a policy's output hides the bug that produced it, and the
    STM32 is the one place the limit is enforced.

    Raises ValueError for the wrong length or a non-finite value - neither can
    mean anything, and a NaN on the wire only counts toward a fault.
    """
    vals = [float(v) for v in values]
    if len(vals) != NUM_JOINTS:
        raise ValueError(f"residual_rad has {len(vals)} values, expected {NUM_JOINTS}")

    bad = [JOINT_NAMES[i] for i, v in enumerate(vals) if not math.isfinite(v)]
    if bad:
        raise ValueError("non-finite residual for " + ", ".join(bad))

    turns = [v / TWO_PI for v in vals]
    over = [i for i, t in enumerate(turns) if abs(t) > RESIDUAL_LIMIT_TURNS]
    return turns, over


def policy_block(state) -> np.ndarray:
    """
    The 46-value observation block, as one float32 array.

    Works on a zeus_msgs/NexusState or a nexus_proto.NexusState - they share
    field names - and returns the same order as the packet's policy block, so
    an observation built on the Pi matches one sliced from raw bytes.

    Raw SI values. Any scaling, clipping or normalisation the policy was
    trained with is still yours to apply.
    """
    parts = []
    for name, count in POLICY_FIELDS:
        value = getattr(state, name)
        parts.append(np.atleast_1d(np.asarray(value, dtype=np.float32)))
    out = np.concatenate(parts)
    assert out.size == sum(n for _, n in POLICY_FIELDS)
    return out


def joint_state_arrays(state) -> Tuple[List[str], List[float], List[float], List[float]]:
    """
    Names, positions, velocities and efforts for a sensor_msgs/JointState.

    From a packet or a message alike. Plain Python floats, which is what the
    unbounded float64[] fields of JointState accept.

    The bolted joints come last, at zero. This build has no waist actuators, so
    the packet says nothing about the waist - but robot_state_publisher needs
    every joint in the URDF to place the parts above it, and leaving them out
    breaks the model in half at the waist. Zero is not a guess here: the waist
    is bolted at the pose the URDF calls zero. Effort is reported as zero too,
    which is what an unpowered joint applies.
    """
    n_bolted = len(BOLTED_JOINT_NAMES)
    return (
        list(JOINT_NAMES) + list(BOLTED_JOINT_NAMES),
        [float(v) for v in state.joint_pos] + [0.0] * n_bolted,
        [float(v) for v in state.joint_vel] + [0.0] * n_bolted,
        [float(v) for v in state.act_torque] + [0.0] * n_bolted,
    )


def names_of(indices: Iterable[int]) -> List[str]:
    return [JOINT_NAMES[i] for i in indices]
