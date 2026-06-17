"""
Zeus Sensor Fusion Node — Contact-Aided InEKF State Estimator.

Fuses:
  • BNO085 IMU data  (orientation, angular velocity, linear acceleration)
  • AS5048A after-spring encoder angles  (4 SEA joints: hip_pitch and knee_pitch)
  • ODrive load_encoder_position         (6 QDD joints: hip_roll, ankle_pitch, waist)
  • 4 mechanical contact switches  (left/right toe & heel, GPIO)

into:
  • World-frame velocity  (body and world frame)
  • Lower-torso height above ground
  • Full pose estimate (position + orientation)
  • Estimated IMU biases

Filter algorithm: Contact-Aided Right-Invariant EKF (Hartley et al. 2019).

Published topics
----------------
/zeus/estimated_height          std_msgs/Float64
/zeus/estimated_velocity        geometry_msgs/TwistWithCovariance
/zeus/estimated_pose            geometry_msgs/PoseWithCovariance
/zeus/contact_state             std_msgs/Bool (latched array, [L_toe, L_heel, R_toe, R_heel])
/zeus/imu_bias_gyro             geometry_msgs/Vector3Stamped
/zeus/imu_bias_accel            geometry_msgs/Vector3Stamped

Subscribed topics
-----------------
/imu_sensor_broadcaster/imu                    sensor_msgs/Imu
/dynamic_joint_states                          control_msgs/DynamicJointState
/contact_state_broadcaster/dynamic_joint_states control_msgs/DynamicJointState
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import (
    PoseWithCovariance, TwistWithCovariance,
    Vector3Stamped, Quaternion,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Bool

from .inekf import ContactAidedRIEKF, InEKFParams
from .kinematics import KinematicsParams, fk_left_foot, fk_right_foot


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('zeus_sensor_fusion')

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self._declare_params()

        # ------------------------------------------------------------------
        # Load kinematics parameters
        # ------------------------------------------------------------------
        kin = KinematicsParams(
            thigh_length=self.get_parameter('thigh_length').value,
            shank_length=self.get_parameter('shank_length').value,
            foot_height=self.get_parameter('foot_height').value,
            left_hip_offset=np.array(self.get_parameter('left_hip_offset').value),
            right_hip_offset=np.array(self.get_parameter('right_hip_offset').value),
        )
        self._kin = kin

        # ------------------------------------------------------------------
        # Load InEKF parameters
        # ------------------------------------------------------------------
        ekf_params = InEKFParams(
            noise_gyro_std=self.get_parameter('noise_gyro_std').value,
            noise_accel_std=self.get_parameter('noise_accel_std').value,
            noise_gyro_bias_std=self.get_parameter('noise_gyro_bias_std').value,
            noise_accel_bias_std=self.get_parameter('noise_accel_bias_std').value,
            noise_contact_vel_std=self.get_parameter('noise_contact_vel_std').value,
            noise_encoder_std=self.get_parameter('noise_encoder_std').value,
            init_orientation_std=self.get_parameter('init_orientation_std').value,
            init_velocity_std=self.get_parameter('init_velocity_std').value,
            init_position_std=self.get_parameter('init_position_std').value,
            init_contact_std=self.get_parameter('init_contact_std').value,
            init_gyro_bias_std=self.get_parameter('init_gyro_bias_std').value,
            init_accel_bias_std=self.get_parameter('init_accel_bias_std').value,
            gravity=np.array(self.get_parameter('gravity').value),
        )
        self._filter = ContactAidedRIEKF(ekf_params)

        # ------------------------------------------------------------------
        # Sensor data caches
        # ------------------------------------------------------------------
        # Effective joint angles for all 10 joints (radians), used by FK.
        # SEA joints (0,2,5,7): populated from after_spring_angle (AS5048A, rad).
        # QDD joints (1,3,4,6,8,9): populated from load_encoder_position × 2π
        #   (ODrive output-shaft turns converted to rad).
        self._encoder_angles: np.ndarray = np.zeros(10)
        self._encoder_ready: bool = False

        # Contact switch states per sensor name → 0.0 / 1.0
        self._switch_raw: dict = {
            'left_toe_switch':   0.0,
            'left_heel_switch':  0.0,
            'right_toe_switch':  0.0,
            'right_heel_switch': 0.0,
        }
        # Debounce counters per switch
        debounce = self.get_parameter('contact_debounce_count').value
        self._debounce_count: int = debounce
        self._debounce_counters: dict = {k: 0 for k in self._switch_raw}
        self._switch_confirmed: dict = {k: False for k in self._switch_raw}

        # Active contact state per side ('left' / 'right')
        self._foot_contact: dict = {'left': False, 'right': False}

        # Pending FK update flags (set when contact is active + encoders fresh)
        self._pending_fk_update: dict = {'left': False, 'right': False}

        # IMU timing
        self._last_imu_stamp: float | None = None

        # ------------------------------------------------------------------
        # Subscriptions
        # ------------------------------------------------------------------
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.create_subscription(
            Imu,
            '/imu_sensor_broadcaster/imu',
            self._imu_callback,
            best_effort,
        )
        self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            DynamicJointState,
            '/contact_state_broadcaster/dynamic_joint_states',
            self._contact_state_callback,
            10,
        )

        # ------------------------------------------------------------------
        # Publishers
        # ------------------------------------------------------------------
        self._pub_height = self.create_publisher(Float64, '/zeus/estimated_height', 10)
        self._pub_velocity = self.create_publisher(
            TwistWithCovariance, '/zeus/estimated_velocity', 10)
        self._pub_pose = self.create_publisher(
            PoseWithCovariance, '/zeus/estimated_pose', 10)
        self._pub_contact = self.create_publisher(Bool, '/zeus/contact_state', 10)
        self._pub_bias_gyro = self.create_publisher(
            Vector3Stamped, '/zeus/imu_bias_gyro', 10)
        self._pub_bias_accel = self.create_publisher(
            Vector3Stamped, '/zeus/imu_bias_accel', 10)

        # ------------------------------------------------------------------
        # Publish timer
        # ------------------------------------------------------------------
        rate = self.get_parameter('publish_rate').value
        self.create_timer(1.0 / rate, self._publish_estimates)

        self.get_logger().info(
            f'Zeus Sensor Fusion started — thigh={kin.thigh_length:.3f} m, '
            f'shank={kin.shank_length:.3f} m, foot_h={kin.foot_height:.3f} m')

    # ------------------------------------------------------------------
    # Parameter declaration
    # ------------------------------------------------------------------

    def _declare_params(self):
        # Kinematics
        self.declare_parameter('thigh_length',      0.30)
        self.declare_parameter('shank_length',      0.30)
        self.declare_parameter('foot_height',       0.05)
        self.declare_parameter('left_hip_offset',   [0.0,  0.05, 0.0])
        self.declare_parameter('right_hip_offset',  [0.0, -0.05, 0.0])

        # InEKF process noise
        self.declare_parameter('noise_gyro_std',          0.002)
        self.declare_parameter('noise_accel_std',         0.04)
        self.declare_parameter('noise_gyro_bias_std',     0.001)
        self.declare_parameter('noise_accel_bias_std',    0.001)
        self.declare_parameter('noise_contact_vel_std',   0.05)
        self.declare_parameter('noise_encoder_std',       0.0175)

        # InEKF initial covariance
        self.declare_parameter('init_orientation_std',    0.5236)
        self.declare_parameter('init_velocity_std',       1.0)
        self.declare_parameter('init_position_std',       0.1)
        self.declare_parameter('init_contact_std',        0.1)
        self.declare_parameter('init_gyro_bias_std',      0.005)
        self.declare_parameter('init_accel_bias_std',     0.05)

        # Gravity
        self.declare_parameter('gravity', [0.0, 0.0, -9.81])

        # Contact
        self.declare_parameter('contact_debounce_count', 3)

        # Output
        self.declare_parameter('publish_rate', 100.0)

    # ------------------------------------------------------------------
    # IMU callback — main filter predict loop
    # ------------------------------------------------------------------

    def _imu_callback(self, msg: Imu):
        # Compute dt
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_imu_stamp is None:
            self._last_imu_stamp = stamp_sec
            return
        dt = stamp_sec - self._last_imu_stamp
        self._last_imu_stamp = stamp_sec

        if dt <= 0.0 or dt > 0.5:
            return

        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])

        # --- Predict ---
        self._filter.predict(omega, accel, dt)

        # --- Update active contacts with fresh FK ---
        if self._encoder_ready:
            for side in ('left', 'right'):
                if self._pending_fk_update[side]:
                    self._do_fk_update(side)
                    self._pending_fk_update[side] = False

    # ------------------------------------------------------------------
    # Joint state callback — cache encoder angles
    # ------------------------------------------------------------------

    # SEA joints publish after_spring_angle (radians from AS5048A).
    # QDD joints only publish load_encoder_position (ODrive turns); multiply by 2π for radians.
    _SEA_JOINTS = frozenset({'joint_0', 'joint_2', 'joint_5', 'joint_7'})
    _QDD_JOINTS = frozenset({'joint_1', 'joint_3', 'joint_4', 'joint_6', 'joint_8', 'joint_9'})
    _JOINT_IDX = {
        'joint_0': 0, 'joint_1': 1, 'joint_2': 2, 'joint_3': 3,
        'joint_4': 4, 'joint_5': 5, 'joint_6': 6, 'joint_7': 7,
        'joint_8': 8, 'joint_9': 9,
    }
    _TWO_PI = 2.0 * np.pi

    def _joint_state_callback(self, msg: DynamicJointState):
        for name, interface_values in zip(msg.joint_names, msg.interface_values):
            if name not in self._JOINT_IDX:
                continue
            idx = self._JOINT_IDX[name]
            for iface, val in zip(
                    interface_values.interface_names, interface_values.values):
                if name in self._SEA_JOINTS and iface == 'after_spring_angle':
                    self._encoder_angles[idx] = val
                elif name in self._QDD_JOINTS and iface == 'load_encoder_position':
                    # ODrive reports output-shaft position in turns; convert to radians.
                    self._encoder_angles[idx] = val * self._TWO_PI

        self._encoder_ready = True

        # Mark FK update pending for active contacts
        for side in ('left', 'right'):
            if self._foot_contact[side]:
                self._pending_fk_update[side] = True

    # ------------------------------------------------------------------
    # Contact switch callback — debounce and manage contact events
    # ------------------------------------------------------------------

    def _contact_state_callback(self, msg: DynamicJointState):
        for name, interface_values in zip(msg.joint_names, msg.interface_values):
            if name not in self._switch_raw:
                continue
            for iface, val in zip(
                    interface_values.interface_names, interface_values.values):
                if iface == 'contact':
                    self._switch_raw[name] = val

        # Aggregate toe+heel into per-side contact with debounce
        # A side is "in contact" if EITHER toe OR heel switch is active.
        for side, toe_key, heel_key in (
            ('left',  'left_toe_switch',  'left_heel_switch'),
            ('right', 'right_toe_switch', 'right_heel_switch'),
        ):
            raw_active = (
                self._switch_raw[toe_key] > 0.5 or
                self._switch_raw[heel_key] > 0.5
            )
            # Increment or reset debounce counter
            key = side
            if raw_active:
                self._debounce_counters[key] = min(
                    self._debounce_counters.get(key, 0) + 1,
                    self._debounce_count + 1,
                )
            else:
                self._debounce_counters[key] = max(
                    self._debounce_counters.get(key, 0) - 1, 0)

            confirmed = self._debounce_counters[key] >= self._debounce_count

            if confirmed and not self._foot_contact[side]:
                # Rising edge: foot just made contact
                self._on_contact_made(side)
            elif not confirmed and self._foot_contact[side]:
                # Falling edge: foot just lifted
                self._on_contact_lost(side)

    def _on_contact_made(self, side: str):
        """Handle a confirmed new ground contact."""
        self._foot_contact[side] = True
        if self._encoder_ready:
            B_p_BC, J_p = self._compute_fk(side)
            self._filter.add_contact(side, B_p_BC, J_p)
            self.get_logger().info(
                f'{side} foot contact — FK: {B_p_BC.round(3)} m (body frame)')

    def _on_contact_lost(self, side: str):
        """Handle a confirmed contact loss."""
        self._foot_contact[side] = False
        self._pending_fk_update[side] = False
        self._filter.remove_contact(side)
        self.get_logger().info(f'{side} foot lifted')

    def _do_fk_update(self, side: str):
        """Apply a FK measurement update to the InEKF for the given contact."""
        B_p_BC, J_p = self._compute_fk(side)
        self._filter.update_contact(side, B_p_BC, J_p)

    def _compute_fk(self, side: str):
        """Compute FK for a given side using current encoder angles."""
        if side == 'left':
            return fk_left_foot(self._encoder_angles, self._kin)
        return fk_right_foot(self._encoder_angles, self._kin)

    # ------------------------------------------------------------------
    # Publish estimates
    # ------------------------------------------------------------------

    def _publish_estimates(self):
        stamp = self.get_clock().now().to_msg()

        # --- Height ---
        height_msg = Float64()
        height_msg.data = self._filter.get_height()
        self._pub_height.publish(height_msg)

        # --- Velocity ---
        vel_world = self._filter.get_velocity_world()
        vel_body = self._filter.get_velocity_body()
        P = self._filter.get_covariance()
        # Velocity covariance is the 3×3 block at rows/cols 3:6 (world-frame velocity)
        vel_cov = P[3:6, 3:6]

        twist_msg = TwistWithCovariance()
        twist_msg.twist.linear.x = float(vel_body[0])
        twist_msg.twist.linear.y = float(vel_body[1])
        twist_msg.twist.linear.z = float(vel_body[2])
        # Flatten 6×6 covariance (linear only; angular is zeroed)
        twist_cov = np.zeros(36)
        twist_cov[0]  = float(vel_cov[0, 0])
        twist_cov[1]  = float(vel_cov[0, 1])
        twist_cov[2]  = float(vel_cov[0, 2])
        twist_cov[6]  = float(vel_cov[1, 0])
        twist_cov[7]  = float(vel_cov[1, 1])
        twist_cov[8]  = float(vel_cov[1, 2])
        twist_cov[12] = float(vel_cov[2, 0])
        twist_cov[13] = float(vel_cov[2, 1])
        twist_cov[14] = float(vel_cov[2, 2])
        twist_msg.covariance = twist_cov.tolist()
        self._pub_velocity.publish(twist_msg)

        # --- Pose ---
        pos_world = self._filter.get_position_world()
        R = self._filter.get_rotation()
        q = _rotation_matrix_to_quaternion(R)
        pos_cov = P[6:9, 6:9]

        pose_msg = PoseWithCovariance()
        pose_msg.pose.position.x = float(pos_world[0])
        pose_msg.pose.position.y = float(pos_world[1])
        pose_msg.pose.position.z = float(pos_world[2])
        pose_msg.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        pose_cov = np.zeros(36)
        pose_cov[0]  = float(pos_cov[0, 0])
        pose_cov[7]  = float(pos_cov[1, 1])
        pose_cov[14] = float(pos_cov[2, 2])
        pose_msg.covariance = pose_cov.tolist()
        self._pub_pose.publish(pose_msg)

        # --- Contact state (Bool) ---
        contact_msg = Bool()
        contact_msg.data = bool(self._foot_contact['left'] or self._foot_contact['right'])
        self._pub_contact.publish(contact_msg)

        # --- IMU biases ---
        bias = self._filter.get_bias()
        bg_msg = Vector3Stamped()
        bg_msg.header.stamp = stamp
        bg_msg.header.frame_id = 'imu_link'
        bg_msg.vector.x = float(bias[0])
        bg_msg.vector.y = float(bias[1])
        bg_msg.vector.z = float(bias[2])
        self._pub_bias_gyro.publish(bg_msg)

        ba_msg = Vector3Stamped()
        ba_msg.header.stamp = stamp
        ba_msg.header.frame_id = 'imu_link'
        ba_msg.vector.x = float(bias[3])
        ba_msg.vector.y = float(bias[4])
        ba_msg.vector.z = float(bias[5])
        self._pub_bias_accel.publish(ba_msg)


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a 3×3 rotation matrix to quaternion [x, y, z, w]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
