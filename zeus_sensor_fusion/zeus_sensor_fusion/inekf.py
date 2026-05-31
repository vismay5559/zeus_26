"""
Contact-Aided Right-Invariant Extended Kalman Filter (RIEKF) for Zeus.

Implements Sections 5–8 + Appendix A of:
  Hartley et al. 2019, "Contact-Aided Invariant Extended Kalman Filtering
  for Robot State Estimation."

State:
  X  ∈ SE_{N+2}(3)  — (5+N)×(5+N) matrix containing R, v, p, d_contacts
  θ  ∈ R^6          — IMU bias vector [b_g (3); b_a (3)]
  P  ∈ R^{m×m}      — right-invariant error covariance, m = 3(N+3) + 6

The filter handles dynamic contact addition/removal as the robot's feet
touch and lift from the ground.

Key equations referenced:
  Prediction  : (24), (50), (51), (58), (61)
  Update      : (19), (20), (29)
  Add contact : (31), (32)
  Remove      : (30)
  Bias model  : (24), (25), (28)
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np

from .lie_group import (
    Gamma0, Gamma1, Gamma2,
    exp_SEK3, inv_SEK3, Adjoint,
    skew, state_dim, state_transition_right,
)


# ---------------------------------------------------------------------------
# Filter parameters
# ---------------------------------------------------------------------------

@dataclass
class InEKFParams:
    # Process noise standard deviations
    noise_gyro_std: float = 0.002          # rad/s
    noise_accel_std: float = 0.04          # m/s²
    noise_gyro_bias_std: float = 0.001     # rad/s²
    noise_accel_bias_std: float = 0.001    # m/s³
    noise_contact_vel_std: float = 0.05    # m/s  (foot-slip model)
    noise_encoder_std: float = 0.0175      # rad  (~1 deg)

    # Initial covariance standard deviations
    init_orientation_std: float = 0.5236   # rad (30 deg)
    init_velocity_std: float = 1.0         # m/s
    init_position_std: float = 0.1         # m
    init_contact_std: float = 0.1          # m
    init_gyro_bias_std: float = 0.005      # rad/s
    init_accel_bias_std: float = 0.05      # m/s²

    # Gravity in world frame [x, y, z]  (Z-up → g = [0, 0, -9.81])
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))


# ---------------------------------------------------------------------------
# Helper: build continuous process noise Q̄ (equation 28)
# ---------------------------------------------------------------------------

def _build_Q_bar(R_bar: np.ndarray, N: int, contact_ids: List[str], params: InEKFParams) -> np.ndarray:
    """
    Continuous-time right-invariant noise covariance  Q̄_t = Ad_{X̄} Cov(w) Ad_{X̄}^T.

    For the right-invariant world-centric case the noise enters as:
        Q̄_t[pose] = Ad_{X̄} · diag(Σ_g, Σ_a, Σ_cv, Σ_cv, ...) · Ad_{X̄}^T
        Q̄_t[bias] = diag(Σ_bg, Σ_ba)

    We approximate with identity Adjoint for the bias-only terms and use the
    full Ad for the pose terms, following the paper's imperfect-InEKF approach.
    """
    n_contacts = N
    # dim of pose tangent space
    n_pose = 3 * (n_contacts + 3)
    n_total = n_pose + 6
    Q_bar = np.zeros((n_total, n_total))

    # Rotation noise → index 0:3
    sig_g = params.noise_gyro_std**2
    Q_bar[0:3, 0:3] = sig_g * np.eye(3)

    # Velocity noise → index 3:6
    sig_a = params.noise_accel_std**2
    Q_bar[3:6, 3:6] = sig_a * (R_bar @ R_bar.T)   # R Σ_a R^T

    # Position noise (zero in the deterministic model — driven by velocity)
    # Q_bar[6:9, 6:9] = 0  (already zero)

    # Contact velocity noise (foot slip model) — each contact block
    sig_cv = params.noise_contact_vel_std**2
    for k in range(n_contacts):
        base = 9 + 3 * k
        Q_bar[base: base + 3, base: base + 3] = sig_cv * (R_bar @ R_bar.T)

    # Gyro bias noise
    sig_bg = params.noise_gyro_bias_std**2
    Q_bar[n_pose:     n_pose + 3, n_pose:     n_pose + 3] = sig_bg * np.eye(3)

    # Accel bias noise
    sig_ba = params.noise_accel_bias_std**2
    Q_bar[n_pose + 3: n_pose + 6, n_pose + 3: n_pose + 6] = sig_ba * np.eye(3)

    return Q_bar


# ---------------------------------------------------------------------------
# Main filter class
# ---------------------------------------------------------------------------

class ContactAidedRIEKF:
    """
    Contact-Aided Right-Invariant Extended Kalman Filter.

    Usage
    -----
    filter = ContactAidedRIEKF(params)

    # At ~400 Hz (IMU callback):
    filter.predict(omega_meas, accel_meas, dt)

    # When contact switches change:
    filter.add_contact('left', B_p_BC, J_p)
    filter.remove_contact('right')

    # While foot is in contact (each IMU cycle, after predict):
    filter.update_contact('left', B_p_BC, J_p)

    # Read results:
    height   = filter.get_height()
    vel_body = filter.get_velocity_body()
    vel_world= filter.get_velocity_world()
    """

    def __init__(self, params: Optional[InEKFParams] = None):
        self.params = params or InEKFParams()

        # Contact registry: maps contact_id ('left'/'right') → column index in X
        # Column index 0 = v, 1 = p, 2+ = contacts
        self._contacts: Dict[str, int] = {}
        self._next_contact_col: int = 2     # first contact occupies column 2 in X

        # State matrix X ∈ SE_{0+2}(3) = SE_2(3)   (5×5 with no contacts initially)
        N = 0
        d = state_dim(N)
        self._X = np.eye(d)

        # IMU bias  θ = [b_g; b_a]
        self._theta = np.zeros(6)

        # Covariance (right-invariant error + bias)
        self._P = self._init_covariance(N)

        # Gravity vector (world frame)
        self._g = self.params.gravity.copy()

    # -----------------------------------------------------------------------
    # Public state accessors
    # -----------------------------------------------------------------------

    def get_rotation(self) -> np.ndarray:
        """Body→world rotation matrix R."""
        return self._X[0:3, 0:3].copy()

    def get_velocity_world(self) -> np.ndarray:
        """Linear velocity in world frame (m/s)."""
        return self._X[0:3, 3].copy()

    def get_position_world(self) -> np.ndarray:
        """IMU/lower-torso position in world frame (m)."""
        return self._X[0:3, 4].copy()

    def get_velocity_body(self) -> np.ndarray:
        """Linear velocity expressed in body/IMU frame (m/s)."""
        return self._X[0:3, 0:3].T @ self._X[0:3, 3]

    def get_height(self) -> float:
        """
        Z-coordinate of the IMU in world frame (m).
        With ground defined as z=0 and Z pointing up, this is the actual
        height of the lower torso above the ground.
        """
        return float(self._X[2, 4])

    def get_bias(self) -> np.ndarray:
        """Estimated IMU bias [b_g (3); b_a (3)]."""
        return self._theta.copy()

    def get_covariance(self) -> np.ndarray:
        """Full covariance matrix of the right-invariant error + bias."""
        return self._P.copy()

    def get_active_contacts(self) -> List[str]:
        """List of currently active contact IDs ('left', 'right')."""
        return list(self._contacts.keys())

    @property
    def N(self) -> int:
        """Number of active contact points."""
        return len(self._contacts)

    # -----------------------------------------------------------------------
    # Prediction step  (equations 24, 50, 51, 58, 61)
    # -----------------------------------------------------------------------

    def predict(
        self,
        omega_meas: np.ndarray,
        accel_meas: np.ndarray,
        dt: float,
    ) -> None:
        """
        Propagate state and covariance forward by dt using IMU measurements.

        Parameters
        ----------
        omega_meas  Raw gyro reading  ω̃  (rad/s), body frame
        accel_meas  Raw accel reading ã  (m/s²), body frame
        dt          Time step (s)
        """
        if dt <= 0.0 or dt > 0.5:
            return

        R = self._X[0:3, 0:3]
        v = self._X[0:3, 3]
        p = self._X[0:3, 4]

        # Bias-correct inputs  (equation 24)
        omega_bar = omega_meas - self._theta[0:3]
        a_bar = accel_meas - self._theta[3:6]

        phi = omega_bar * dt
        G0 = Gamma0(phi)
        G1 = Gamma1(phi)
        G2 = Gamma2(phi)

        # Propagate rotation, velocity, position  (equation 50)
        R_new = R @ G0
        v_new = v + R @ G1 @ a_bar * dt + self._g * dt
        p_new = p + v * dt + R @ G2 @ a_bar * dt**2 + 0.5 * self._g * dt**2

        self._X[0:3, 0:3] = R_new
        self._X[0:3, 3] = v_new
        self._X[0:3, 4] = p_new
        # Contact positions d_k remain unchanged (stationary contact assumption)

        # Propagate covariance  (equations 51, 61)
        contact_positions = [self._X[0:3, 3 + 2 + k]
                             for k in range(self.N)]
        Phi = state_transition_right(
            omega_bar, a_bar, R, v, p, contact_positions, dt)

        R_bar = R_new
        Q_bar = _build_Q_bar(R_bar, self.N, list(self._contacts.keys()), self.params)
        Q_d = Phi @ Q_bar @ Phi.T * dt      # discrete noise approximation  (eq. 61)
        self._P = Phi @ self._P @ Phi.T + Q_d

    # -----------------------------------------------------------------------
    # Measurement update  (equations 19, 20, 29)
    # -----------------------------------------------------------------------

    def update_contact(
        self,
        contact_id: str,
        B_p_BC: np.ndarray,
        J_p: np.ndarray,
    ) -> None:
        """
        Apply a right-invariant forward-kinematic measurement update.

        Parameters
        ----------
        contact_id  'left' or 'right'
        B_p_BC      Contact position in body/IMU frame from FK  (3-vector, m)
        J_p         3×4 analytical/numerical Jacobian of B_p_BC w.r.t. joint angles
        """
        if contact_id not in self._contacts:
            return

        col_idx = self._contacts[contact_id]   # column in X for this contact
        R_bar = self._X[0:3, 0:3]

        # -------------------------------------------------------------------
        # Innovation  (right-invariant form, equation 18)
        # The observation vector Y = [h_p(ã); 0; 1; −1] (padded to full dim)
        # X^{−1} Y selects the contact innovation:
        #   innovation = R_bar^T (d_k − p_bar) − B_p_BC
        # After linearisation this simplifies to H ξ where H = [0  0  −I  I  0]
        # -------------------------------------------------------------------
        N = self.N
        d = state_dim(N)
        Xinv = inv_SEK3(self._X, N)

        # Build the b-vector in full state space  (equation 18)
        # b^T = [0 0 0  |  0  |  1  |  −1  |  zeros for other contacts]
        b = np.zeros(d)
        b[4] = 1.0                    # position column +1
        b[3 + col_idx] = -1.0         # contact column  −1

        innov_full = Xinv @ b         # Y − b  in the right-invariant sense
        innovation = innov_full[0:3]  # only the 3-vector position part

        # -------------------------------------------------------------------
        # Observation matrix H  (equation 20)
        # H selects  ξ_p (position error) and ξ_d_k (contact error)
        # Indices:  φ=0:3, v=3:6, p=6:9, d_k=9+3*(k):9+3*(k+1), bias=n_pose:
        # -------------------------------------------------------------------
        N_contacts = self.N
        n_pose = 3 * (N_contacts + 3)
        n_total = n_pose + 6

        # Which contact slot?
        k = col_idx - 2              # contact list index (col 0=v, 1=p, 2+=contacts)
        H = np.zeros((3, n_total))
        H[0:3, 6:9]                   = -np.eye(3)   # −ξ_p
        H[0:3, 9 + 3 * k: 12 + 3 * k] =  np.eye(3)  # +ξ_d_k

        # -------------------------------------------------------------------
        # Measurement noise  N̄ = R̄ J_p Σ_enc J_p^T R̄^T  (equation 20)
        # -------------------------------------------------------------------
        Sigma_enc = (self.params.noise_encoder_std**2) * np.eye(4)
        N_bar = R_bar @ J_p @ Sigma_enc @ J_p.T @ R_bar.T

        # -------------------------------------------------------------------
        # Kalman gain  (equation 29)
        # -------------------------------------------------------------------
        S = H @ self._P @ H.T + N_bar
        K = self._P @ H.T @ np.linalg.solve(S.T, np.eye(3)).T   # P H^T S^{-1}

        # Split gain into pose part and bias part
        K_xi = K[:n_pose, :]
        K_zeta = K[n_pose:, :]

        # -------------------------------------------------------------------
        # State update  (equation 29)
        # -------------------------------------------------------------------
        delta_xi = K_xi @ innovation              # tangent-space correction
        delta_zeta = K_zeta @ innovation          # bias correction

        delta_X = exp_SEK3(delta_xi, N_contacts)
        self._X = delta_X @ self._X
        self._theta += delta_zeta

        # Covariance update — Joseph form for numerical stability
        I_KH = np.eye(n_total) - K @ H
        self._P = I_KH @ self._P @ I_KH.T + K @ N_bar @ K.T

    # -----------------------------------------------------------------------
    # Contact management
    # -----------------------------------------------------------------------

    def add_contact(
        self,
        contact_id: str,
        B_p_BC: np.ndarray,
        J_p: np.ndarray,
    ) -> None:
        """
        Augment state with a new contact point.

        Parameters
        ----------
        contact_id  'left' or 'right'
        B_p_BC      Contact position in body frame from FK  (3-vector, m)
        J_p         3×4 Jacobian of B_p_BC
        """
        if contact_id in self._contacts:
            return  # already tracking

        R = self._X[0:3, 0:3]
        p = self._X[0:3, 4]

        # Initialize contact world position via FK  (equation 31)
        d_new = p + R @ B_p_BC

        # --- Augment X ---
        N_old = self.N
        d_old = state_dim(N_old)
        d_new_dim = state_dim(N_old + 1)
        X_new = np.eye(d_new_dim)
        X_new[0: d_old, 0: d_old] = self._X
        X_new[0:3, d_old] = d_new          # new contact column
        self._X = X_new

        # Register the contact — it occupies column index (2 + N_old) in X
        col_idx = 2 + N_old
        self._contacts[contact_id] = col_idx

        # --- Augment P  (equation 32) ---
        N_new = self.N
        n_pose_old = 3 * (N_old + 3)
        n_total_old = n_pose_old + 6
        n_pose_new = 3 * (N_new + 3)
        n_total_new = n_pose_new + 6

        # F maps old error state to new: ξ^new_d = ξ^old_p + R̄ J_p w^α
        # Augmentation matrix F (equation 32)
        F = np.zeros((n_total_new, n_total_old))
        # Copy existing pose rows (φ, v, p, old contacts) identity mapping
        F[0: n_pose_old, 0: n_pose_old] = np.eye(n_pose_old)
        # New contact error ← existing position error (ξ^new_d = ξ^old_p)
        F[n_pose_old: n_pose_old + 3, 6:9] = np.eye(3)
        # Bias rows pass through unchanged
        F[n_pose_new: n_total_new, n_pose_old: n_total_old] = np.eye(6)

        # Noise matrix G (equation 32)
        Sigma_enc = (self.params.noise_encoder_std**2) * np.eye(4)
        G = np.zeros((n_total_new, 3))
        G[n_pose_old: n_pose_old + 3, :] = R @ J_p   # Noise enters via FK Jacobian

        self._P = F @ self._P @ F.T + G @ (Sigma_enc[:3, :3]) @ G.T

    def remove_contact(self, contact_id: str) -> None:
        """
        Remove a contact point by marginalizing it from the state  (equation 30).

        Parameters
        ----------
        contact_id  'left' or 'right'
        """
        if contact_id not in self._contacts:
            return

        col_idx = self._contacts.pop(contact_id)
        k = col_idx - 2          # index within the contact list (0-based)

        N_old = self.N + 1       # was N_old before removal
        N_new = self.N

        d_old = state_dim(N_old)
        d_new = state_dim(N_new)

        # --- Shrink X: remove the contact column/row ---
        X_new = np.eye(d_new)
        # Rows/cols: 0:3 (R block), 3:(3+contact_col) (v,p, earlier contacts),
        #            then skip col_idx, then later contacts
        keep_cols = [i for i in range(d_old) if i != col_idx]
        keep_rows = keep_cols
        X_new = self._X[np.ix_(keep_rows, keep_cols)]
        # Fix the lower-right identity block
        X_new[3:, 3:] = np.eye(d_new - 3)
        # Restore rotation and translation columns
        X_new[0:3, 0:3] = self._X[0:3, 0:3]
        new_col = 3
        for old_col in [c for c in range(3, d_old) if c != col_idx]:
            X_new[0:3, new_col] = self._X[0:3, old_col]
            new_col += 1
        self._X = X_new

        # Remap remaining contact column indices (shift those after the removed one)
        for cid in list(self._contacts.keys()):
            if self._contacts[cid] > col_idx:
                self._contacts[cid] -= 1

        # --- Shrink P: marginalization  (equation 30) ---
        n_pose_old = 3 * (N_old + 3)
        n_total_old = n_pose_old + 6
        n_pose_new = 3 * (N_new + 3)
        n_total_new = n_pose_new + 6

        # Marginalization matrix M selects all rows except the removed contact block
        removed_rows_start = 9 + 3 * k    # error-state index of removed contact
        keep = (
            list(range(0, removed_rows_start)) +
            list(range(removed_rows_start + 3, n_pose_old)) +
            list(range(n_pose_old, n_total_old))
        )
        M = np.zeros((n_total_new, n_total_old))
        for new_i, old_i in enumerate(keep):
            M[new_i, old_i] = 1.0

        self._P = M @ self._P @ M.T

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _init_covariance(self, N: int) -> np.ndarray:
        """Build initial covariance diagonal for N contacts."""
        p = self.params
        n_pose = 3 * (N + 3)
        n_total = n_pose + 6
        diag = np.zeros(n_total)
        # φ, v, p blocks
        diag[0:3] = p.init_orientation_std**2
        diag[3:6] = p.init_velocity_std**2
        diag[6:9] = p.init_position_std**2
        # Contact blocks
        for k in range(N):
            base = 9 + 3 * k
            diag[base: base + 3] = p.init_contact_std**2
        # Bias blocks
        diag[n_pose:     n_pose + 3] = p.init_gyro_bias_std**2
        diag[n_pose + 3: n_pose + 6] = p.init_accel_bias_std**2
        return np.diag(diag)

    def reset(self) -> None:
        """Reset the filter to the initial state (both feet unknown, zero bias)."""
        self._contacts.clear()
        N = 0
        d = state_dim(N)
        self._X = np.eye(d)
        self._theta = np.zeros(6)
        self._P = self._init_covariance(N)
