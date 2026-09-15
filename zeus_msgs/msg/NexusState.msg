# One state packet from the STM32, as published on /zeus/state.
#
# Field for field the 444-byte USB packet (link_proto.h nexus_state_t, protocol
# v6), with the same names as zeus_link.nexus_proto.NexusState. Values are
# exactly what the board sent - no scaling, no unit changes, no filtering.
#
# header.stamp is when the Pi RECEIVED the packet. seq and timestamp_us are the
# board's own clock, and seq is the one to trust for timing: it advances by
# exactly one per 1 kHz control tick, so seq/1000 is seconds since the STM32
# booted and a gap in seq is a packet that was never published.
#
# /zeus/state is latest-wins. The link node always publishes the newest packet
# and skips any that arrived while it was busy, so a slow subscriber sees fresh
# data rather than a growing backlog. Watch seq for gaps if every packet matters.

# ---- joint index: every [10] array below uses this order -------------------
# index = bus * 5 + (node - 1); bus 0 = FDCAN1, bus 1 = FDCAN2.
uint8 J_LEFT_HIP_PITCH=0
uint8 J_LEFT_HIP_ROLL=1
uint8 J_LEFT_KNEE_PITCH=2
uint8 J_LEFT_ANKLE_PITCH=3
uint8 J_WAIST_ROLL=4
uint8 J_RIGHT_HIP_PITCH=5
uint8 J_RIGHT_HIP_ROLL=6
uint8 J_RIGHT_KNEE_PITCH=7
uint8 J_RIGHT_ANKLE_PITCH=8
uint8 J_WAIST_PITCH=9

# ---- contact[4] order, and bits in `contacts` ------------------------------
uint8 CONTACT_LEFT_TOE=0
uint8 CONTACT_LEFT_HEEL=1
uint8 CONTACT_RIGHT_TOE=2
uint8 CONTACT_RIGHT_HEEL=3
uint8 CONTACTS_LEFT_FOOT_BIT=16
uint8 CONTACTS_RIGHT_FOOT_BIT=32

# ---- safety_state ----------------------------------------------------------
uint8 SAFETY_BOOT=0
uint8 SAFETY_IDLE=1
uint8 SAFETY_ARMED=2
uint8 SAFETY_FAULT=3

# ---- fused_valid -----------------------------------------------------------
uint8 FUSION_INVALID=0
uint8 FUSION_CONVERGING=1
uint8 FUSION_OK=2

# ---- health bits (a set bit is a fault) ------------------------------------
uint8 HEALTH_IMU=1
uint8 HEALTH_ENC=2
uint8 HEALTH_CAN1=4
uint8 HEALTH_CAN2=8
uint8 HEALTH_LINK=16
uint8 HEALTH_TIMING=32

# ---- fk_valid bits, stream_flags bits --------------------------------------
uint8 FK_RIGHT_VALID=1
uint8 FK_LEFT_VALID=2
uint8 STREAM_GAIT_LIVE=1
# Set by the single-leg bench test (STM32 NEXUS_MODE_LEG_CAN), not the robot loop.
# Only the joints that test drives are real; estimator, IMU, springs and
# contacts are not running. Commands sent back are ignored by the board.
uint8 STREAM_LEG_TEST=2

std_msgs/Header header

uint32 seq                      # 1 kHz tick counter on the STM32
uint32 timestamp_us             # STM32 1 MHz counter, wraps every ~71 min

# ==== policy block: 52 values, in this order =================================
# zeus_link.convert.policy_block(msg) returns them as one float32 array.
float32 pelvis_z                # m, height above the stance ground
float32[4] quat                 # w,x,y,z body->world, fused estimate
float32[3] gyro                 # rad/s, body frame
float32[3] vel_hdg              # m/s, heading frame: lateral, forward, vertical
float32[10] joint_pos           # rad, output side
float32[10] joint_vel           # rad/s, output side
float32[4] spring_angle         # rad, SPRING DEFLECTION, not a joint angle
float32[10] ref_angle           # rad, output side: stored gait at `phase`
float32[4] contact              # 0.0 / 1.0, debounced, CONTACT_* order
float32[2] foot_z               # m, world. [0] right, [1] left. NaN if invalid
float32 phase                   # 0..1 stride clock
# ==== end policy block ========================================================

# ---- raw IMU (BNO085's own fusion and readings) ----------------------------
float32[4] imu_quat             # w,x,y,z
float32[3] imu_accel            # m/s^2, specific force, INCLUDES gravity
float32[3] imu_gyro             # rad/s
uint32 imu_seq

# ---- actuators ---------------------------------------------------------------
float32[10] act_torque          # Nm, ODrive estimate
uint32[10] act_error            # raw ODrive axis_error bitmask
uint8[10] act_state             # raw ODrive axis_state (8 = CLOSED_LOOP_CONTROL)
uint8[10] act_flags             # bit 0 telemetry fresh, bit 1 heartbeat fresh

# ---- estimator internals -----------------------------------------------------
float32[3] fused_pos            # m, world. [2] is pelvis_z; [0],[1] drift
float32[3] fused_vel            # m/s, world frame
float32[3] fused_gyro_bias      # rad/s
float32[3] fused_accel_bias     # m/s^2

# ---- diagnostics -------------------------------------------------------------
uint32 overruns                 # control ticks missed, cumulative
uint32 usb_dropped              # state packets the board could not send
uint16[2] can_dropped           # CAN TX frames dropped, per bus
uint16 loop_us_max              # worst control cycle since the last packet
uint16 enc_stalls               # spring-encoder SPI transfers abandoned
uint8[2] can_bus_off            # bus-off events per bus, saturating at 255
uint8 stream_flags              # STREAM_*
uint16[2] contact_ticks         # ticks each foot has held its state
uint8 enc_valid                 # bit per spring encoder
uint8 contacts                  # switch bits 0..3, plus CONTACTS_*_FOOT_BIT
uint8 fused_valid               # FUSION_*
uint8 health                    # HEALTH_* bits
uint8 fk_valid                  # FK_*_VALID bits, one per foot_z entry
uint8 safety_state              # SAFETY_*
