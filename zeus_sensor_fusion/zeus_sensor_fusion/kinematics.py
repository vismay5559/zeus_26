"""
Zeus bipedal robot forward kinematics.

Joint mapping (symmetric legs, confirmed hardware layout):
    joint_0  = left_hip_pitch   (can0, node 1)
    joint_1  = left_hip_roll    (can0, node 2)
    joint_2  = left_knee_pitch  (can0, node 3)
    joint_3  = left_ankle_pitch (can0, node 4)
    joint_4  = waist_pitch      (can0, node 5)  — NOT used in foot FK
    joint_5  = right_hip_pitch  (can1, node 1)
    joint_6  = right_hip_roll   (can1, node 2)
    joint_7  = right_knee_pitch (can1, node 3)
    joint_8  = right_ankle_pitch(can1, node 4)
    joint_9  = waist_roll       (can1, node 5)  — NOT used in foot FK

The IMU is located in the lower torso (base_link / body frame).
Waist joints connect the lower torso to the upper body and therefore do NOT
appear in the kinematic chain from the lower torso to the feet.

Rotation conventions (right-hand, Z-up world frame):
    Hip pitch  → rotation about +Y axis  (sagittal plane, forward/back)
    Hip roll   → rotation about +X axis  (frontal plane, left/right)
    Knee pitch → rotation about +Y axis
    Ankle pitch→ rotation about +Y axis

Link lengths are loaded from kinematics.yaml and must be measured on the
physical robot.  Placeholder values of 0.30 m are provided for testing.
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple


@dataclass
class KinematicsParams:
    thigh_length: float = 0.30       # hip joint → knee joint (m)
    shank_length: float = 0.30       # knee joint → ankle joint (m)
    foot_height: float = 0.05        # ankle joint → ground contact point (m)
    left_hip_offset: np.ndarray = None   # body-frame offset to left hip joint
    right_hip_offset: np.ndarray = None  # body-frame offset to right hip joint

    def __post_init__(self):
        if self.left_hip_offset is None:
            self.left_hip_offset = np.array([0.0, 0.05, 0.0])
        if self.right_hip_offset is None:
            self.right_hip_offset = np.array([0.0, -0.05, 0.0])


# ---------------------------------------------------------------------------
# Elementary rotation matrices
# ---------------------------------------------------------------------------

def Rx(theta: float) -> np.ndarray:
    """4×4 homogeneous rotation about the X axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1.0, 0.0,  0.0, 0.0],
        [0.0,   c,   -s, 0.0],
        [0.0,   s,    c, 0.0],
        [0.0, 0.0,  0.0, 1.0],
    ])


def Ry(theta: float) -> np.ndarray:
    """4×4 homogeneous rotation about the Y axis."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [  c, 0.0,   s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [ -s, 0.0,   c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def Tx(dx: float, dy: float, dz: float) -> np.ndarray:
    """4×4 homogeneous pure translation."""
    T = np.eye(4)
    T[0, 3] = dx
    T[1, 3] = dy
    T[2, 3] = dz
    return T


# ---------------------------------------------------------------------------
# Single-leg forward kinematics
# ---------------------------------------------------------------------------

def _fk_foot_transform(
    q_hip_pitch: float,
    q_hip_roll: float,
    q_knee_pitch: float,
    q_ankle_pitch: float,
    hip_offset: np.ndarray,
    thigh: float,
    shank: float,
    foot_h: float,
) -> np.ndarray:
    """
    Compute the 4×4 homogeneous transform from the body frame (lower torso /
    IMU frame) to the foot contact point.

    Chain:
      T_body → hip_offset → hip_pitch(Y) → hip_roll(X)
            → thigh(−Z)  → knee_pitch(Y) → shank(−Z)
            → ankle_pitch(Y) → foot contact(−Z)
    """
    T = (
        Tx(*hip_offset)
        @ Ry(q_hip_pitch)
        @ Rx(q_hip_roll)
        @ Tx(0.0, 0.0, -thigh)
        @ Ry(q_knee_pitch)
        @ Tx(0.0, 0.0, -shank)
        @ Ry(q_ankle_pitch)
        @ Tx(0.0, 0.0, -foot_h)
    )
    return T


def _numerical_jacobian(
    q: np.ndarray,
    hip_offset: np.ndarray,
    params: KinematicsParams,
    eps: float = 1e-6,
) -> np.ndarray:
    """
    3×4 numerical Jacobian of the foot contact position w.r.t. joint angles.
    Used to compute the measurement noise covariance N̄ = R̄ J_p Σ_enc J_p^T R̄^T.
    """
    def p(q_):
        T = _fk_foot_transform(
            q_[0], q_[1], q_[2], q_[3],
            hip_offset, params.thigh_length, params.shank_length, params.foot_height,
        )
        return T[0:3, 3]

    J = np.zeros((3, 4))
    p0 = p(q)
    for i in range(4):
        dq = np.zeros(4)
        dq[i] = eps
        J[:, i] = (p(q + dq) - p0) / eps
    return J


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def fk_left_foot(
    q_all: np.ndarray,
    params: KinematicsParams,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Forward kinematics for the LEFT foot contact point.

    Parameters
    ----------
    q_all   Full 10-DOF joint angle vector (after-spring encoders, radians).
            Index mapping: [0]=L_hip_pitch, [1]=L_hip_roll, [2]=L_knee_pitch,
                           [3]=L_ankle_pitch, [4]=waist_pitch, [5]=R_hip_pitch,
                           [6]=R_hip_roll, [7]=R_knee_pitch, [8]=R_ankle_pitch,
                           [9]=waist_roll
    params  Link length parameters.

    Returns
    -------
    B_p_BC  3-vector: contact position expressed in the body/IMU frame.
    J_p     3×4 Jacobian of B_p_BC w.r.t. [hip_pitch, hip_roll, knee_pitch, ankle_pitch].
    """
    q_leg = q_all[[0, 1, 2, 3]]
    T = _fk_foot_transform(
        q_leg[0], q_leg[1], q_leg[2], q_leg[3],
        params.left_hip_offset,
        params.thigh_length, params.shank_length, params.foot_height,
    )
    B_p_BC = T[0:3, 3]
    J_p = _numerical_jacobian(q_leg, params.left_hip_offset, params)
    return B_p_BC, J_p


def fk_right_foot(
    q_all: np.ndarray,
    params: KinematicsParams,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Forward kinematics for the RIGHT foot contact point.

    Parameters
    ----------
    q_all   Full 10-DOF joint angle vector (after-spring encoders, radians).
    params  Link length parameters.

    Returns
    -------
    B_p_BC  3-vector: contact position in body/IMU frame.
    J_p     3×4 Jacobian w.r.t. [hip_pitch, hip_roll, knee_pitch, ankle_pitch].
    """
    q_leg = q_all[[5, 6, 7, 8]]
    T = _fk_foot_transform(
        q_leg[0], q_leg[1], q_leg[2], q_leg[3],
        params.right_hip_offset,
        params.thigh_length, params.shank_length, params.foot_height,
    )
    B_p_BC = T[0:3, 3]
    J_p = _numerical_jacobian(q_leg, params.right_hip_offset, params)
    return B_p_BC, J_p


def fk_both_feet(
    q_all: np.ndarray,
    params: KinematicsParams,
) -> dict:
    """
    Convenience wrapper — returns a dict with both feet.

    Returns
    -------
    {
      'left':  (B_p_BC_left,  J_p_left),
      'right': (B_p_BC_right, J_p_right),
    }
    """
    return {
        'left': fk_left_foot(q_all, params),
        'right': fk_right_foot(q_all, params),
    }


def nominal_standing_height(params: KinematicsParams) -> float:
    """
    Geometric lower-bound on standing height when both knees and ankles are
    straight (all joint angles = 0).  Useful as an initial height guess.
    """
    return params.thigh_length + params.shank_length + params.foot_height
