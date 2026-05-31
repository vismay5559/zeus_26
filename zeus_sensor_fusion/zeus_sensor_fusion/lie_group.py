"""
SE_{N+2}(3) Lie group mathematics for the contact-aided InEKF.

All notation follows Hartley et al. 2019:
  "Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation"

The group SE_K(3) is the group of K direct isometries containing a rotation
R ∈ SO(3) and K translation vectors p_1, ..., p_K ∈ R^3.  For the Zeus biped
the state matrix is:

    X = [ R   v   p   d_L   d_R ]    (7×7 when both feet are in contact)
        [ 0   1   0    0     0  ]
        [ 0   0   1    0     0  ]
        [ 0   0   0    1     0  ]
        [ 0   0   0    0     1  ]

where  R  = body→world rotation,
       v  = body linear velocity in world frame,
       p  = body position in world frame,
       d  = world-frame position of each active contact point.

References: equations (6), (7), (49), (58), (63)-(66) in the paper.
"""

import numpy as np


# ---------------------------------------------------------------------------
# SO(3) primitives
# ---------------------------------------------------------------------------

def skew(v: np.ndarray) -> np.ndarray:
    """3-vector → 3×3 skew-symmetric matrix  (v)_×."""
    v = v.ravel()
    return np.array([
        [ 0.0,  -v[2],  v[1]],
        [ v[2],  0.0,  -v[0]],
        [-v[1],  v[0],   0.0],
    ])


def unskew(S: np.ndarray) -> np.ndarray:
    """3×3 skew-symmetric matrix → 3-vector."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def _angle_and_axis(phi: np.ndarray):
    """Return (θ, φ_hat) where θ = ||φ|| and φ_hat is the unit axis."""
    theta = float(np.linalg.norm(phi))
    return theta, phi


def Gamma0(phi: np.ndarray) -> np.ndarray:
    """
    SO(3) exponential map  exp(φ)  =  Γ₀(φ).
    Equation (49) in the paper.
    """
    theta = float(np.linalg.norm(phi))
    S = skew(phi)
    if theta < 1e-9:
        return np.eye(3) + S + 0.5 * S @ S
    return np.eye(3) + (np.sin(theta) / theta) * S + \
        ((1.0 - np.cos(theta)) / theta**2) * (S @ S)


def Gamma1(phi: np.ndarray) -> np.ndarray:
    """
    Left Jacobian of SO(3)  J_l(φ) = Γ₁(φ).
    Equation (49) in the paper.
    """
    theta = float(np.linalg.norm(phi))
    S = skew(phi)
    if theta < 1e-9:
        return np.eye(3) + 0.5 * S + (1.0 / 6.0) * S @ S
    return np.eye(3) + \
        ((1.0 - np.cos(theta)) / theta**2) * S + \
        ((theta - np.sin(theta)) / theta**3) * (S @ S)


def Gamma2(phi: np.ndarray) -> np.ndarray:
    """
    Double-integral of SO(3) exponential  Γ₂(φ).
    Equation (49) in the paper.
    """
    theta = float(np.linalg.norm(phi))
    S = skew(phi)
    if theta < 1e-9:
        return 0.5 * np.eye(3) + (1.0 / 6.0) * S + (1.0 / 24.0) * S @ S
    return 0.5 * np.eye(3) + \
        ((theta - np.sin(theta)) / theta**3) * S + \
        ((theta**2 + 2.0 * np.cos(theta) - 2.0) / (2.0 * theta**4)) * (S @ S)


# ---------------------------------------------------------------------------
# SE_K(3) operations
# ---------------------------------------------------------------------------

def state_dim(N: int) -> int:
    """Matrix dimension of SE_{N+2}(3): rotation + velocity + position + N contacts."""
    return N + 5


def wedge(xi: np.ndarray, N: int) -> np.ndarray:
    """
    Hat operator (·)^  :  R^{3(N+2)} → se_{N+2}(3)  Lie algebra matrix.
    xi layout: [φ (3), ξ_v (3), ξ_p (3), ξ_d1 (3), ..., ξ_dN (3)]
    Equation (6) / (64) in the paper.
    """
    d = state_dim(N)
    Xi = np.zeros((d, d))
    phi = xi[0:3]
    Xi[0:3, 0:3] = skew(phi)
    for k in range(N + 2):
        Xi[0:3, 3 + k] = xi[3 + 3 * k: 6 + 3 * k]
    return Xi


def vee(Xi: np.ndarray, N: int) -> np.ndarray:
    """
    Vee operator (·)^∨  :  se_{N+2}(3) → R^{3(N+2)}.
    Inverse of wedge.
    """
    xi = np.zeros(3 * (N + 3))
    xi[0:3] = unskew(Xi[0:3, 0:3])
    for k in range(N + 2):
        xi[3 + 3 * k: 6 + 3 * k] = Xi[0:3, 3 + k]
    return xi


def exp_SEK3(xi: np.ndarray, N: int) -> np.ndarray:
    """
    SE_{N+2}(3) exponential map.
    Equation (66) in the paper.
    xi layout: [φ, ξ_v, ξ_p, ξ_d1, ..., ξ_dN]
    """
    d = state_dim(N)
    X = np.eye(d)
    phi = xi[0:3]
    R = Gamma0(phi)
    G1 = Gamma1(phi)
    X[0:3, 0:3] = R
    for k in range(N + 2):
        p_k = xi[3 + 3 * k: 6 + 3 * k]
        X[0:3, 3 + k] = G1 @ p_k
    return X


def inv_SEK3(X: np.ndarray, N: int) -> np.ndarray:
    """
    SE_{N+2}(3) group inverse.
    X^{-1} follows from the block structure: R^T and −R^T p_k.
    """
    d = state_dim(N)
    Xinv = np.eye(d)
    R = X[0:3, 0:3]
    RT = R.T
    Xinv[0:3, 0:3] = RT
    for k in range(N + 2):
        Xinv[0:3, 3 + k] = -RT @ X[0:3, 3 + k]
    return Xinv


def Adjoint(X: np.ndarray, N: int) -> np.ndarray:
    """
    Adjoint representation  Ad_X  :  se_{N+2}(3) → se_{N+2}(3).
    Equation (7) / (63) in the paper.
    dim = 3(N+3).
    """
    R = X[0:3, 0:3]
    n_cols = N + 2          # number of translation columns (v, p, d_1, ..., d_N)
    size = 3 * (N + 3)      # total tangent-space dimension (3 for φ + 3 per column)
    Ad = np.zeros((size, size))
    Ad[0:3, 0:3] = R
    for k in range(n_cols):
        p_k = X[0:3, 3 + k]
        row = 3 + 3 * k
        Ad[row: row + 3, 0:3] = skew(p_k) @ R
        Ad[row: row + 3, row: row + 3] = R
    return Ad


# ---------------------------------------------------------------------------
# Discrete state-transition matrix  Φ^r  (right-invariant, world-centric)
# ---------------------------------------------------------------------------

def state_transition_right(
    omega_bar: np.ndarray,
    a_bar: np.ndarray,
    R: np.ndarray,
    v: np.ndarray,
    p: np.ndarray,
    contact_positions: list,
    dt: float,
) -> np.ndarray:
    """
    Right-invariant state-transition matrix  Φ^r(t_{k+1}, t_k).
    Equation (58) in the paper.

    State order (right-invariant error, world-centric):
        [φ (3), δv (3), δp (3), δd_1 (3), ..., δd_N (3), δb_g (3), δb_a (3)]
    Total dimension = 3*(N+3) + 6 = 3N + 15.

    Parameters
    ----------
    omega_bar   Bias-corrected angular velocity  ω̃ − b_g  (rad/s)
    a_bar       Bias-corrected linear acceleration  ã − b_a  (m/s²)
    R           Current rotation matrix (body→world)
    v           Current world-frame velocity
    p           Current world-frame position
    contact_positions   List of world-frame contact positions [d_L, d_R, ...]
    dt          Time step (s)
    """
    N = len(contact_positions)
    # Tangent-space size: 3(N+3) for pose+contacts, 6 for biases
    n_pose = 3 * (N + 3)
    n_total = n_pose + 6
    Phi = np.eye(n_total)

    phi = omega_bar * dt
    G0 = Gamma0(phi)
    G1 = Gamma1(phi)
    G2 = Gamma2(phi)

    v_next = v + R @ G1 @ a_bar * dt
    p_next = p + v * dt + R @ G2 @ a_bar * dt**2

    g = np.array([0.0, 0.0, -9.81])  # will be overridden by caller if needed

    # Φ^r_{11}: rotation block (G0^T)
    Phi[0:3, 0:3] = G0.T

    # Φ^r_{21}: (g)_× Δt  — velocity ← rotation coupling
    Phi[3:6, 0:3] = skew(g) * dt

    # Φ^r_{31}: 0.5 (g)_× Δt²
    Phi[6:9, 0:3] = 0.5 * skew(g) * dt**2

    # Φ^r_{32}: I Δt  — position ← velocity coupling
    Phi[6:9, 3:6] = np.eye(3) * dt

    # Contact blocks Φ^r_{(d_k,d_k)} = I (contacts are constant)
    for k in range(N):
        base = 9 + 3 * k
        Phi[base: base + 3, base: base + 3] = np.eye(3)

    # Bias coupling into rotation  Φ^r_{1,bg} = −R̄ Γ₁ Δt
    bg_col = n_pose
    Phi[0:3, bg_col: bg_col + 3] = -R @ G1 * dt

    # Bias coupling into velocity  Φ^r_{2,bg} and Φ^r_{2,ba}
    ba_col = n_pose + 3
    Phi[3:6, bg_col: bg_col + 3] = -skew(v_next) @ R @ G1 * dt + R @ _Psi1(omega_bar, a_bar, dt)
    Phi[3:6, ba_col: ba_col + 3] = -R @ G1 * dt

    # Bias coupling into position
    Phi[6:9, bg_col: bg_col + 3] = -skew(p_next) @ R @ G1 * dt + R @ _Psi2(omega_bar, a_bar, dt)
    Phi[6:9, ba_col: ba_col + 3] = -R @ G2 * dt**2

    # Bias coupling into each contact  Φ^r_{dk,bg}
    for k in range(N):
        base = 9 + 3 * k
        d_k = contact_positions[k]
        Phi[base: base + 3, bg_col: bg_col + 3] = -skew(d_k) @ R @ G1 * dt

    # Bias blocks are identity (biases propagate as random walk)
    Phi[n_pose: n_pose + 6, n_pose: n_pose + 6] = np.eye(6)

    return Phi


def _Psi1(omega_bar, a_bar, dt):
    """Ψ₁ approximation: (ā)_× Γ₂(−ω̄Δt) Δt²  (simplified first-order term)."""
    phi = -omega_bar * dt
    return skew(a_bar) @ Gamma2(phi) * dt**2


def _Psi2(omega_bar, a_bar, dt):
    """Ψ₂ approximation: (ā)_× Γ₃(−ω̄Δt) Δt³  (simplified first-order term)."""
    phi = -omega_bar * dt
    theta = float(np.linalg.norm(phi))
    S = skew(phi)
    if theta < 1e-9:
        G3 = (1.0 / 6.0) * np.eye(3)
    else:
        G3 = (1.0 / 6.0) * np.eye(3) + \
            ((theta**2 / 2.0 - 1.0 + np.cos(theta)) / theta**4) * S + \
            ((theta - np.sin(theta) - theta**3 / 6.0) / theta**5) * (S @ S)
    return skew(a_bar) @ G3 * dt**3
