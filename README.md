# Zeus — Bipedal Robot ROS 2 Workspace

Zeus is a 10-DOF bipedal robot controlled from a Raspberry Pi 4B running ROS 2 Jazzy.
This workspace contains every package needed to go from raw sensor hardware to
a full state estimate that a walking controller can consume.

---

## What the Robot Has

| Hardware | Details |
|----------|---------|
| **Compute** | Raspberry Pi 4B, `linux-raspi-realtime` kernel (PREEMPT_RT) |
| **Actuators** | 10 × ODrive S1 brushless motor drivers, 5 per CAN bus |
| **CAN-FD HAT** | Waveshare dual-channel CAN-FD HAT (MCP2518FD chip, Mode A) — SPI0 + SPI1 |
| **After-spring encoders** | 4 × AS5048A 13-bit magnetic encoders, SPI3 daisy-chain on `/dev/spidev3.0` (SEA joints only) |
| **IMU** | Adafruit BNO085, SHTP protocol over hardware UART `/dev/ttyAMA0` at 3 Mbaud |
| **Contact switches** | 4 × mechanical switches wired directly to Raspberry Pi GPIO pins |
| **CAN buses** | 2 × SocketCAN (`can_odrive` / `can1`), CAN-FD at 5 Mbps data rate |

> **PREEMPT_RT kernel is required.** The controller manager runs a FIFO-50 RT thread at 1 kHz.
> Without the realtime kernel, SPI interrupt handlers can hold locks for 8+ ms and cause
> constant overruns. Install with: `sudo apt install linux-raspi-realtime`

### CAN Bus Note — TX vs RX speeds

The Waveshare HAT uses MCP2518FD chips over SPI. Under sustained CAN-FD TX at 5 Mbps
(1 kHz × 5 actuators), the MCP2518FD SPI driver experiences an interrupt storm that deadlocks
the Pi. The workaround:

- **Pi → ODrive (Set_Input_Pos):** Classic CAN frames at **1 Mbps** — stable at 1 kHz
- **ODrive → Pi (encoder, torque):** CAN-FD frames at **5 Mbps** data phase — received fine
- The CAN interface is still brought up with `fd on` so the socket can receive FD frames

### Leg and Joint Mapping

```
Joint index │ Actuator          │ CAN bus    │ ODrive node ID │ Type │ Encoder
────────────┼───────────────────┼────────────┼────────────────┼──────┼──────────────────────
  joint_0   │ left_hip_pitch    │ can_odrive │  1             │ SEA  │ AS5048A #0 (SPI3)
  joint_1   │ left_hip_roll     │ can_odrive │  2             │ QDD  │ ODrive only
  joint_2   │ left_knee_pitch   │ can_odrive │  3             │ SEA  │ AS5048A #1 (SPI3)
  joint_3   │ left_ankle_pitch  │ can_odrive │  4             │ QDD  │ ODrive only
  joint_4   │ waist_pitch       │ can_odrive │  5             │ QDD  │ ODrive only
  joint_5   │ right_hip_pitch   │ can1       │  1             │ SEA  │ AS5048A #2 (SPI3)
  joint_6   │ right_hip_roll    │ can1       │  2             │ QDD  │ ODrive only
  joint_7   │ right_knee_pitch  │ can1       │  3             │ SEA  │ AS5048A #3 (SPI3)
  joint_8   │ right_ankle_pitch │ can1       │  4             │ QDD  │ ODrive only
  joint_9   │ waist_roll        │ can1       │  5             │ QDD  │ ODrive only
```

SEA = Series Elastic Actuator — spring deflection measured by AS5048A gives `after_spring_angle`.
QDD = Quasi Direct Drive — no spring; output angle equals gearbox output reported by ODrive (`load_encoder_position × 2π rad`).

### CAN Interface Naming

The Waveshare HAT has two MCP2518FD channels. Kernel enumeration order is
non-deterministic across reboots. A udev rule pins the first channel to a stable name:

```
/etc/udev/rules.d/99-can-odrive.rules:
  SUBSYSTEM=="net", ACTION=="add", KERNELS=="spi0.0", NAME="can_odrive"
```

All launch files and scripts use `can_odrive` as the interface name.

### Contact Switch GPIO Pins (BCM numbering)

```
Switch            │ GPIO pin │ Physical pin
──────────────────┼──────────┼─────────────
left_toe_switch   │    4     │  Pin 7
left_heel_switch  │    5     │  Pin 29
right_toe_switch  │    6     │  Pin 31
right_heel_switch │   13     │  Pin 33
```

---

## How Data Flows

```
Physical hardware
  ├── ODrive S1 (CAN) ──────────────────┐
  ├── AS5048A encoders (SPI) ────────────┤
  ├── BNO085 IMU (UART) ────────────────┤──► zeus_hardware_interface
  └── Mechanical switches (GPIO sysfs) ──┘           │
                                                      │ ROS 2 state interfaces
                                                      ▼
                                           ros2_control controller_manager
                                                      │
                          ┌───────────────────────────┼──────────────────┐
                          ▼                           ▼                  ▼
               joint_state_broadcaster   imu_sensor_broadcaster   contact_state_broadcaster
                          │                           │                  │
                          └───────────────┬───────────┘                  │
                                          ▼                              │
                                zeus_sensor_fusion ◄─────────────────────┘
                                          │
                              /zeus/estimated_height
                              /zeus/estimated_velocity
                              /zeus/estimated_pose
```

ROS 2 control vocabulary:
- **command interface** — a value ROS writes to hardware (target joint angle).
- **state interface** — a value hardware reports back to ROS (sensor reading).

---

## Interface Count: 38 state + 10 command

### Command Interfaces (10)

```
joint_0/target_actuator_angle
joint_1/target_actuator_angle
...
joint_9/target_actuator_angle
```

### State Interfaces (38)

```
SEA joints — after_spring_angle + load_encoder_position + torque_estimate (4 joints × 3 = 12):
  joint_0/after_spring_angle   joint_0/load_encoder_position   joint_0/torque_estimate
  joint_2/after_spring_angle   joint_2/load_encoder_position   joint_2/torque_estimate
  joint_5/after_spring_angle   joint_5/load_encoder_position   joint_5/torque_estimate
  joint_7/after_spring_angle   joint_7/load_encoder_position   joint_7/torque_estimate

QDD joints — load_encoder_position + torque_estimate only (6 joints × 2 = 12):
  joint_1/load_encoder_position   joint_1/torque_estimate
  joint_3/load_encoder_position   joint_3/torque_estimate
  joint_4/load_encoder_position   joint_4/torque_estimate
  joint_6/load_encoder_position   joint_6/torque_estimate
  joint_8/load_encoder_position   joint_8/torque_estimate
  joint_9/load_encoder_position   joint_9/torque_estimate

IMU (10):
  imu/orientation.x   imu/orientation.y   imu/orientation.z   imu/orientation.w
  imu/linear_acceleration.x   .y   .z
  imu/angular_velocity.x      .y   .z

Mechanical contact switches (4):
  left_toe_switch/contact
  left_heel_switch/contact
  right_toe_switch/contact
  right_heel_switch/contact
```

### Target Update Rates

```
controller_manager loop:    1000 Hz   (FIFO-50 RT thread, PREEMPT_RT kernel required)
Actuator CAN commands:      1000 Hz   (classic CAN 1 Mbps TX; ODrive configured to broadcast at 1 kHz)
AS5048A encoder reads:      1000 Hz   (SPI3 burst, 8 bytes per cycle — 4 SEA joints)
GPIO contact switch reads:  1000 Hz   (pre-opened sysfs fd — no open() overhead)
ODrive CAN telemetry RX:    1000 Hz   (CAN-FD 5 Mbps from ODrive → Pi)
BNO085 IMU UART stream:      400 Hz   (3 Mbaud, FIFO-buffered, no clock stretching)
Sensor fusion publish:       100 Hz
```

---

## Package Layout

```
zeus_26/
├── zeus/                       Metapackage (groups all other packages)
├── zeus_can_interface/         Low-level SocketCAN driver for ODrive CAN messages
├── zeus_hardware_interface/    ROS 2 control hardware plugin — the main hardware bridge
├── zeus_description/           Robot URDF/xacro and ros2_control configuration
├── zeus_bringup/               Launch files, controller YAML, and utility scripts
├── zeus_control_interface/     Python placeholder for RL policy commands
├── zeus_sensor_fusion/         Contact-aided InEKF state estimator
└── zeus_gazebo/                Simulation scaffold (not yet active)
```

---

## Package Details

### `zeus_can_interface`

A thin C++ library wrapping Linux SocketCAN.

- Opens a raw CAN socket on any named interface (`can_odrive`, `can1`, ...).
- Enables `CAN_RAW_FD_FRAMES` so the socket can receive ODrive CAN-FD broadcasts.
- `send_position_target(node_id, position, vel_ff)` — sends ODrive `Set_Input_Pos` (0x00C)
  as a **classic CAN frame** (1 Mbps). Includes velocity feedforward in bytes 4–5.
- `send_axis_state(node_id, state)` — sends `Set_Axis_State` (0x007).
- `read_frame(frame)` — non-blocking read; accepts both classic (16 B) and FD (72 B) frames.
- ODrive CANSimple message IDs decoded:
  - `0x009 Get_Encoder_Estimates` — position (rev) and velocity (rev/s).
  - `0x01C Get_Torques` — torque target and torque estimate (Nm).

Key files:
- [zeus_can_interface/include/zeus_can_interface/socketcan.hpp](zeus_can_interface/include/zeus_can_interface/socketcan.hpp)
- [zeus_can_interface/src/socketcan.cpp](zeus_can_interface/src/socketcan.cpp)

---

### `zeus_hardware_interface`

The ROS 2 control `SystemInterface` plugin. This is where all sensors and actuators live.

Key files:
- [zeus_hardware_interface/include/zeus_hardware_interface/zeus_system.hpp](zeus_hardware_interface/include/zeus_hardware_interface/zeus_system.hpp)
- [zeus_hardware_interface/src/zeus_system.cpp](zeus_hardware_interface/src/zeus_system.cpp)
- [zeus_hardware_interface/src/encoder_utils.cpp](zeus_hardware_interface/src/encoder_utils.cpp) — AS5048A SPI reads
- [zeus_hardware_interface/src/sensor_utils.cpp](zeus_hardware_interface/src/sensor_utils.cpp) — BNO085 SHTP/UART reads

#### Lifecycle

| Stage | What happens |
|-------|-------------|
| `on_init()` | Reads xacro hardware params; allocates all state/command arrays (commands initialised to NaN); detects sensor elements from URDF |
| `on_configure()` | Opens CAN sockets, encoder SPI, IMU UART; exports GPIO pins via sysfs and pre-opens value file descriptors |
| `on_activate()` | Marks hardware ready |
| `read()` | 1. Reads 4 AS5048A encoders (SEA joints) → `after_spring_angle`. 2. Drains ODrive CAN frames → `load_encoder_position`, `torque_estimate` (all joints). 3. Reads BNO085 SHTP packets → IMU states. 4. Reads GPIO switches → `contact` states |
| `write()` | For each joint: passes `hw_commands_[i]` **directly** to `send_position_target()` with velocity feedforward computed as `(cmd[n] - cmd[n-1]) × 1000 Hz`. No smoothing filter — the upstream commander already delivers a smooth 1 kHz interpolated stream. Skips joints where `hw_commands_[i]` is NaN (controller not yet active). |
| `on_deactivate()` | Stops issuing commands |
| `on_cleanup()` | Closes all fds; unexports GPIO pins; closes SPI/CAN sockets |

> **NaN initialisation:** `hw_commands_` is initialised to NaN. `forward_command_controller`
> also sets command interfaces to NaN on activate. `write()` skips any joint still at NaN,
> preventing garbage CAN frames until the first real command arrives.

#### AS5048A Encoder Chain

4 encoders daisy-chained on **SPI3** (`/dev/spidev3.0`), covering the 4 SEA joints only
(left/right hip_pitch and knee_pitch). SPI3 is used because the CAN-FD HAT in Mode A
physically owns SPI0 (can_odrive) and SPI1 (can1) for its CAN links.
SPI3 must be enabled by adding `dtoverlay=spi3-1cs` to `/boot/firmware/config.txt`.

QDD joints (hip_roll, ankle_pitch, waist) have no spring and therefore no AS5048A encoder;
their output angle is read directly from the ODrive via CAN (`load_encoder_position`, turns).

Each encoder returns a 13-bit angle (0–8191 counts = 0–2π radians). The driver reads all 4
in one 8-byte SPI burst, applies parity checking, and runs a small first-order low-pass
filter (α = 0.2) to reduce noise.

SPI3 wiring:
```
MOSI → GPIO2  (Pin 3)
MISO → GPIO1  (Pin 28)
SCLK → GPIO3  (Pin 5)
CS0  → GPIO0  (Pin 27)   master chip-select for the daisy-chain
```

#### BNO085 IMU

The BNO085 is driven over **hardware UART** (`/dev/ttyAMA0`). SPI0 and SPI1 are occupied by
the CAN-FD HAT, so the IMU uses the dedicated UART on GPIO14/15.

The SHTP packet framing is identical to the SPI variant. The driver opens `/dev/ttyAMA0`
at 3 Mbaud in raw mode, streams SHTP packets from the UART RX FIFO, and accumulates partial
packets in an internal buffer between `read()` calls.

Three report types enabled at startup:
- Rotation vector → `orientation.{x,y,z,w}`
- Linear acceleration (gravity-removed) → `linear_acceleration.{x,y,z}`
- Gyroscope → `angular_velocity.{x,y,z}`

UART wiring:
```
TX (Pi → IMU RX) → GPIO14  (Pin 8)
RX (Pi ← IMU TX) → GPIO15  (Pin 10)
RESET            → GPIO25  (Pin 22)   optional but recommended
```

Default config in `zeus_ros2_control.xacro`:
```
/dev/ttyAMA0   3000000 baud   RESET=GPIO25   report_interval=2500µs (400 Hz)
```

#### Mechanical Contact Switches

The 4 switches are wired directly to GPIO pins via Linux sysfs:

1. **`on_configure()`**: exports each pin, sets direction to `in`, opens `/sys/class/gpio/gpioN/value`.
2. **`read()`**: `lseek(fd, 0, SEEK_SET); read(fd, buf, 1)` — ~1 µs latency, no `open()` per cycle.
3. **`on_cleanup()`**: closes fds, unexports pins.

---

### `zeus_description`

URDF and ros2_control xacro files.

Key files:
- [zeus_description/urdf/zeus_urdf.xacro](zeus_description/urdf/zeus_urdf.xacro) — full robot geometry
- [zeus_description/urdf/zeus_ros2_control.xacro](zeus_description/urdf/zeus_ros2_control.xacro) — full hardware plugin config
- [zeus_description/urdf/single_joint_test_robot.xacro](zeus_description/urdf/single_joint_test_robot.xacro) — single joint test robot
- [zeus_description/urdf/single_joint_test.xacro](zeus_description/urdf/single_joint_test.xacro) — single joint hardware config

---

### `zeus_bringup`

Launch files, controller YAML, and utility scripts.

#### Controller configs

| File | Purpose |
|------|---------|
| [zeus_bringup/config/zeus_controllers.yaml](zeus_bringup/config/zeus_controllers.yaml) | Full robot — all 10 joints |
| [zeus_bringup/config/single_joint_controllers.yaml](zeus_bringup/config/single_joint_controllers.yaml) | Single joint test — 1 joint, 1000 Hz |

#### Launch files

| File | Purpose |
|------|---------|
| [single_joint_test.launch.py](zeus_bringup/launch/single_joint_test.launch.py) | **Primary test** — single ODrive actuator, full ROS stack at 1 kHz |
| [hardware.launch.py](zeus_bringup/launch/hardware.launch.py) | Full robot stack — all sensors + actuators + all controllers |
| [single_actuator_command_test.launch.py](zeus_bringup/launch/single_actuator_command_test.launch.py) | Older single-joint command test |
| [hardcoded_single_actuator_test.launch.py](zeus_bringup/launch/hardcoded_single_actuator_test.launch.py) | Hardcoded command sequence |
| [sim.launch.py](zeus_bringup/launch/sim.launch.py) | Simulation bringup (Gazebo — not yet active) |

#### Utility scripts

| Script | Purpose |
|--------|---------|
| [single_joint_commander.py](zeus_bringup/scripts/single_joint_commander.py) | Hip Pitch R gait trajectory commander — homes, enters CLOSED_LOOP, runs ±20° trajectory at 1 kHz |
| [trajectory_plotter.py](zeus_bringup/scripts/trajectory_plotter.py) | Records target vs actual ODrive encoder position and saves a PNG plot |
| [odrive_diag.py](zeus_bringup/scripts/odrive_diag.py) | Reads ODrive heartbeat and encoder frames, prints axis state and errors |

---

### `zeus_sensor_fusion`

A full state estimator for the lower torso. Given IMU data, encoder angles, and contact
switch readings, it outputs: velocity, position, orientation, and height above ground.

Key files:
- [zeus_sensor_fusion/zeus_sensor_fusion/lie_group.py](zeus_sensor_fusion/zeus_sensor_fusion/lie_group.py) — SE_{N+2}(3) Lie group math
- [zeus_sensor_fusion/zeus_sensor_fusion/kinematics.py](zeus_sensor_fusion/zeus_sensor_fusion/kinematics.py) — Zeus leg forward kinematics
- [zeus_sensor_fusion/zeus_sensor_fusion/inekf.py](zeus_sensor_fusion/zeus_sensor_fusion/inekf.py) — the filter itself
- [zeus_sensor_fusion/zeus_sensor_fusion/sensor_fusion_node.py](zeus_sensor_fusion/zeus_sensor_fusion/sensor_fusion_node.py) — ROS 2 node
- [zeus_sensor_fusion/config/kinematics.yaml](zeus_sensor_fusion/config/kinematics.yaml) — link lengths **(must be measured)**
- [zeus_sensor_fusion/config/inekf_params.yaml](zeus_sensor_fusion/config/inekf_params.yaml) — filter noise parameters

#### The Algorithm: Contact-Aided Right-Invariant EKF (InEKF)

Based on Hartley et al. 2019, "Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation."

The InEKF exploits the geometric structure of the robot's motion: the error dynamics on the
SE_{N+2}(3) Lie group are *independent of the state estimate*, making it far more robust and
fast-converging than a standard EKF.

**State representation**

```
X  =  ┌ R    v    p    d_L   d_R ┐     (7×7 when both feet are in contact)
       │ 0    1    0    0     0   │
       │ 0    0    1    0     0   │
       │ 0    0    0    1     0   │
       └ 0    0    0    0     1   ┘

R     : 3×3 rotation matrix (body → world)
v     : 3-vector, linear velocity in world frame
p     : 3-vector, position of IMU in world frame
d_L/R : 3-vector, world-frame position of left/right foot contact points
θ     : [b_g; b_a], 6-vector, IMU gyro and accel biases
```

**Forward kinematics chain (per leg)**
```
T_hip_offset → Ry(hip_pitch) → Rx(hip_roll) → translate(0, 0, -thigh_length)
  → Ry(knee_pitch) → translate(0, 0, -shank_length)
  → Ry(ankle_pitch) → translate(0, 0, -foot_height) ──► contact point in body frame
```

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/zeus/estimated_height` | `std_msgs/Float64` | z-coordinate of IMU in world frame (metres) |
| `/zeus/estimated_velocity` | `geometry_msgs/TwistWithCovariance` | Body-frame linear velocity + covariance |
| `/zeus/estimated_pose` | `geometry_msgs/PoseWithCovariance` | Full position + orientation + covariance |
| `/zeus/contact_state` | `std_msgs/Bool` | True if any foot is in confirmed contact |
| `/zeus/imu_bias_gyro` | `geometry_msgs/Vector3Stamped` | Estimated gyroscope bias (rad/s) |
| `/zeus/imu_bias_accel` | `geometry_msgs/Vector3Stamped` | Estimated accelerometer bias (m/s²) |

---

## Build

Full build from workspace root:

```bash
cd ~/zeus_26
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Faster incremental build (hardware + bringup only — needed after C++ changes):

```bash
colcon build --packages-select zeus_can_interface zeus_hardware_interface zeus_description zeus_bringup
source install/setup.bash
```

Python scripts only (no rebuild needed — they are installed as symlinks):

```bash
colcon build --packages-select zeus_bringup
```

---

## Single Joint Test

The single joint test validates the full ROS 2 control stack end-to-end with one ODrive S1
actuator, **without needing any SPI encoders or IMU**. It uses `command_only_mode` in the
hardware interface so state interfaces mirror the commands instead of reading from SPI.

### What runs and what each node does

```
Terminal 1: ros2 launch zeus_bringup single_joint_test.launch.py
  │
  ├── robot_state_publisher
  │     Reads single_joint_test_robot.xacro → publishes /robot_description topic.
  │     ros2_control_node subscribes to this topic to get hardware config.
  │
  ├── ros2_control_node  (controller_manager)
  │     Loads ZeusSystemHardware plugin → opens CAN socket on can_odrive.
  │     Runs FIFO-50 RT thread at 1000 Hz calling read() → update() → write().
  │     In command_only_mode: read() skips SPI/IMU/GPIO, state mirrors command.
  │     write() sends Set_Input_Pos (0x00C) classic CAN to ODrive at 1 kHz.
  │     Also receives ODrive encoder (0x009) and torque (0x01C) CAN-FD frames.
  │
  ├── spawner → forward_command_controller
  │     Type: forward_command_controller/ForwardCommandController
  │     Subscribes to /forward_command_controller/commands (Float64MultiArray).
  │     Writes the received value directly to hw_commands_[0] each cycle.
  │
  └── spawner → joint_state_broadcaster
        Type: joint_state_broadcaster/JointStateBroadcaster
        Reads hw_states_[]: load_encoder_position, torque_estimate, after_spring_angle.
        Publishes /joint_states and /dynamic_joint_states at 1 kHz.

Terminal 2: ros2 run zeus_bringup single_joint_commander.py
  │
  └── HipPitchCommander node
        1. Sends IDLE via raw CAN → clears ODrive errors from previous session.
        2. Reads current ODrive pos_estimate via CAN (0x009) → stores as home (0°).
        3. Sends Set_Controller_Mode: POSITION_CONTROL + INPUT_MODE_PASSTHROUGH.
        4. Sends Set_Axis_State: CLOSED_LOOP_CONTROL.
        5. Verifies via ODrive heartbeat (0x001) that CLOSED_LOOP was accepted.
        6. Publishes Hip Pitch R gait trajectory at 1 kHz to
           /forward_command_controller/commands.
           All positions are relative to home: home + lerp(waypoints, t % 0.9s).
        On Ctrl-C: commands home position, then sends IDLE.
```

### Control loop at 1 kHz — what happens every millisecond

```
1ms tick (FIFO-50 RT thread):

  read()
    └── drain CAN socket (non-blocking) for ODrive frames
          0x009 → hw_states_[0] (load_encoder_position, velocity)
          0x01C → hw_states_[0] (torque_estimate)
          [SPI encoders, IMU, GPIO skipped in command_only_mode]

  update()
    ├── joint_state_broadcaster.update()
    │     copies hw_states_[] → publishes /dynamic_joint_states
    └── forward_command_controller.update()
          reads latest /forward_command_controller/commands message
          writes value → hw_commands_[0]

  write()
    └── hw_commands_[0] is not NaN?
          vel_ff = (hw_commands_[0] - prev_hw_commands_[0]) × 1000  [rev/s]
          send_position_target(node_id=0, position=hw_commands_[0], vel_ff=vel_ff)
          → classic CAN frame (16 bytes) → MCP2518FD → CAN bus → ODrive
```

### URDF / hardware config

The single joint test uses its own minimal xacro:

- **[single_joint_test_robot.xacro](zeus_description/urdf/single_joint_test_robot.xacro)** — robot
  structure (base_link + link_0 + continuous joint_0)
- **[single_joint_test.xacro](zeus_description/urdf/single_joint_test.xacro)** — hardware plugin config:

```xml
<plugin>zeus_hardware_interface/ZeusSystemHardware</plugin>
<param name="can0">can_odrive</param>
<param name="command_only_mode">true</param>    <!-- no SPI encoders needed -->
<param name="num_daisy_encoders">1</param>

<joint name="joint_0">
  <command_interface name="target_actuator_angle"/>
  <state_interface name="after_spring_angle"/>
  <state_interface name="load_encoder_position"/>
  <state_interface name="torque_estimate"/>
  <param name="can_bus">can_odrive</param>
  <param name="node_id">0</param>               <!-- ODrive CAN node ID -->
</joint>
```

### Controller config

[zeus_bringup/config/single_joint_controllers.yaml](zeus_bringup/config/single_joint_controllers.yaml):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 1000          # Hz — the RT control loop rate
    forward_command_controller:
      type: forward_command_controller/ForwardCommandController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

forward_command_controller:
  ros__parameters:
    joints: [joint_0]
    interface_name: target_actuator_angle    # in ODrive turns (revolutions)

joint_state_broadcaster:
  ros__parameters:
    joints: [joint_0]
    interfaces: [after_spring_angle, load_encoder_position, torque_estimate]
```

### Gait trajectory

The commander replays the **Hip Pitch R** column from the gait table.
22 Hz waypoints (every 45 ms) are linearly interpolated to a smooth 1 kHz stream:

```
Waypoints (22 Hz, ±20° range):
  t=0.000s: -20°   t=0.225s:   0°   t=0.450s: +20°
  t=0.675s:   0°   t=0.900s: -20°  (cycle repeats)

Sent to ODrive at 1 kHz as absolute turns:
  position = home_rev + lerp(waypoints, t % 0.9s) / 360
  vel_ff   = Δposition × 1000 Hz  [computed in hardware interface write()]
```

Positions are always relative to **home** — wherever the motor physically was
when the commander started is declared 0°. No need to zero the ODrive manually.

### Tuned ODrive gains

These gains were found to give ~6.7° RMS tracking error at 1.11 Hz, ±20° trajectory
(down from 24.9° with default gains and no velocity feedforward):

```python
# In odrivetool:
odrv0.axis0.controller.config.pos_gain = 25          # Nm/turn
odrv0.axis0.controller.config.vel_gain = 0.42         # Nm/(turn/s)
odrv0.axis0.controller.config.vel_integrator_gain = 0.1
odrv0.save_configuration()
```

### Actuator Fault Detection — Heartbeat + CAN Staleness Watchdog

`zeus_hardware_interface` decodes each ODrive's `Get_Heartbeat` (`0x001`) CAN broadcast and tracks
the age of the last `Get_Encoder_Estimates` (`0x009`) / `Get_Torques` (`0x01C`) frame per joint.
This is in addition to (not a replacement for) each ODrive's own 3 ms CAN watchdog.

**Required ODrive config** (set once per axis via `odrivetool`, alongside the existing
watchdog/brake-resistor config):

```python
odrv0.axis0.config.can.heartbeat_rate_ms = 10   # default is 100 ms — too slow to catch a fault quickly
odrv0.save_configuration()
```

**New state interface per joint:** `actuator_fault` — a reason bitmask, not a plain 0/1 flag.
`0.0` means healthy; a nonzero value tells you exactly which condition(s) tripped:

| Bit | Value | Meaning |
|---|---|---|
| `kFaultBitHeartbeatStale` | `0x1` | No `Get_Heartbeat` frame in >30 ms (3 missed at the required 10 ms rate) |
| `kFaultBitTelemetryStale` | `0x2` | No `Get_Encoder_Estimates`/`Get_Torques` frame in >5 ms (5 missed at the nominal 1 kHz broadcast) |
| `kFaultBitAxisError` | `0x4` | The ODrive's own `Axis_Error` (from its Heartbeat) is nonzero — the raw ODrive error code itself is only in the log line below, not on the topic |
| `kFaultBitTxFailure` | `0x8` | 3+ consecutive failed CAN sends to this joint |

Bits are OR'd together, so e.g. `actuator_fault == 5` (`0x1 | 0x4`) means both a stale heartbeat and
an active ODrive axis error. The full ODrive error code and a human-readable breakdown of which
bits fired are printed via `RCLCPP_WARN` the moment a joint transitions into fault — check the
`ros2_control_node` console/log for the exact ODrive error value.

**Whole-robot full stop:** the instant *any* joint reports a nonzero `actuator_fault` (for any
reason, including a TX-failure-only fault), `write()` withholds position commands from **every**
joint, not just the faulted one — a full stop, not just a one-joint freeze. This is a hard latch:
it does **not** auto-clear when the underlying condition clears, on purpose — a fault serious
enough to trip a full stop should not silently resume unsupervised. The only way to clear it is
to re-activate the hardware component (`reset_actuator_fault_state()`, called from
`on_activate()`), i.e. an explicit operator-driven restart, which also resets every joint's
fault bookkeeping to a clean slate.

This is a deliberately conservative placeholder: with no RL policy loaded yet and no leg
kinematics in the URDF, "stop everything" is the safest default. Once the RL policy and full
URDF are in place, this is the place to revisit for a more tiered response (e.g. controlled
crouch before cutting power, or reacting differently depending on stance vs. swing leg).

**Behavior on fault:** `write()` stops sending new `Set_Input_Pos` commands to that joint only —
it does not send an explicit IDLE/estop. This lets the ODrive's own 3 ms watchdog (already
configured, see below) govern the axis instead of racing it with a second, independently-tuned
reaction. TX failures are tracked but never gate sending, since the send attempt itself is what
lets a transient failure clear — gating on it would permanently latch the fault. A joint recovers
automatically (resumes taking commands) once fresh heartbeat/telemetry frames resume and
`axis_error` reads back 0.

### Step-by-step: bring up and run

```bash
# 1. Bring up CAN interface (do this once after every reboot)
sudo ip link set can_odrive down 2>/dev/null; true
sudo ip link set can_odrive up type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can_odrive txqueuelen 100

# 2. (Optional) Verify ODrive is alive and healthy
source /opt/ros/jazzy/setup.bash && source ~/zeus_26/install/setup.bash
python3 ~/zeus_26/zeus_bringup/scripts/odrive_diag.py

# 3. Terminal 1 — launch the full ROS stack
source /opt/ros/jazzy/setup.bash && source ~/zeus_26/install/setup.bash
ros2 launch zeus_bringup single_joint_test.launch.py
# Wait for: "Successful set up FIFO RT scheduling policy with priority 50"
# and:      "Configured and activated forward_command_controller"

# 4. Terminal 2 — run the gait trajectory commander
source /opt/ros/jazzy/setup.bash && source ~/zeus_26/install/setup.bash
ros2 run zeus_bringup single_joint_commander.py
# Watch for: "[5/5] HOME SET" and "ODrive confirmed: CLOSED_LOOP_CONTROL"
# Motor will oscillate ±20° at ~1.11 Hz

# 5. Terminal 3 — record and plot tracking performance (optional)
source /opt/ros/jazzy/setup.bash && source ~/zeus_26/install/setup.bash
ros2 run zeus_bringup trajectory_plotter.py 10   # records 10 s
# Plot saved to /tmp/trajectory_plot.png
# Copy to laptop: scp vismay@<pi-ip>:/tmp/trajectory_plot.png .
```

### Topics active during single joint test

| Topic | Publisher | Subscriber | Rate |
|-------|-----------|------------|------|
| `/robot_description` | robot_state_publisher | ros2_control_node | once |
| `/forward_command_controller/commands` | single_joint_commander.py | forward_command_controller | 1 kHz |
| `/joint_states` | joint_state_broadcaster | — | 1 kHz |
| `/dynamic_joint_states` | joint_state_broadcaster | single_joint_commander.py | 1 kHz |

---

## Running the Full Hardware Stack

### 1. Bring up CAN buses

```bash
# Primary CAN bus (ODrive joints 0-4)
sudo ip link set can_odrive down 2>/dev/null; true
sudo ip link set can_odrive up type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can_odrive txqueuelen 100

# Secondary CAN bus (ODrive joints 5-9)
sudo ip link set can1 down 2>/dev/null; true
sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
sudo ip link set can1 txqueuelen 100
```

### 2. Launch the full hardware stack

```bash
source /opt/ros/jazzy/setup.bash && source ~/zeus_26/install/setup.bash
ros2 launch zeus_bringup hardware.launch.py
```

### 3. Launch sensor fusion (in a second terminal)

```bash
source /opt/ros/jazzy/setup.bash && source ~/zeus_26/install/setup.bash
ros2 launch zeus_sensor_fusion sensor_fusion.launch.py
```

### 4. Check everything started correctly

```bash
ros2 control list_hardware_interfaces    # expect 38 state + 10 command
ros2 control list_controllers
ros2 topic echo /zeus/estimated_height
ros2 topic echo /dynamic_joint_states
```

---

## Sending Actuator Commands (Full Robot)

The full robot forward command controller expects 10 values (one per joint, in ODrive turns):

```bash
ros2 topic pub /rl_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

> **Use very small values during initial bring-up.** Commands go directly to the ODrive
> position controller. Units are ODrive turns (revolutions), not radians.

---

## Before You Trust the State Estimates

The kinematics config at [zeus_sensor_fusion/config/kinematics.yaml](zeus_sensor_fusion/config/kinematics.yaml) contains **placeholder link lengths**:

```yaml
thigh_length: 0.30   # PLACEHOLDER — measure hip joint to knee joint (metres)
shank_length: 0.30   # PLACEHOLDER — measure knee joint to ankle joint (metres)
foot_height:  0.05   # PLACEHOLDER — measure ankle joint to contact point (metres)
left_hip_offset:  [0.0,  0.05, 0.0]   # PLACEHOLDER
right_hip_offset: [0.0, -0.05, 0.0]   # PLACEHOLDER
```

Measure the actual distances on the physical robot and update this file before
relying on height or velocity estimates.

---

## Useful Debug Commands

```bash
# ROS 2 control
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers

# Check CAN bus is up and ODrive is broadcasting
candump can_odrive                          # should see 0x001, 0x009, 0x01C frames
python3 ~/zeus_26/zeus_bringup/scripts/odrive_diag.py   # prints axis state + errors

# Joint and sensor states
ros2 topic echo /dynamic_joint_states
ros2 topic echo /imu_sensor_broadcaster/imu

# State estimates
ros2 topic echo /zeus/estimated_height
ros2 topic echo /zeus/estimated_velocity
ros2 topic echo /zeus/estimated_pose

# GPIO switch reading
cat /sys/class/gpio/gpio4/value     # left_toe
cat /sys/class/gpio/gpio5/value     # left_heel
cat /sys/class/gpio/gpio6/value     # right_toe
cat /sys/class/gpio/gpio13/value    # right_heel

# IMU UART stream
stty -F /dev/ttyAMA0 3000000 raw
hexdump -C /dev/ttyAMA0 | head -20    # should show SHTP packets at 400 Hz

# Verify SPI3 encoder device
ls /dev/spidev3.0    # requires dtoverlay=spi3-1cs in /boot/firmware/config.txt
```

---

## Hardware Wiring Summary

```
┌─────────────────────────────────────────────────────────────────┐
│                      Raspberry Pi 4B                            │
│                                                                 │
│  SPI0 (GPIO8/9/10/11) ──► CAN-FD HAT ──► can_odrive (CAN-FD)  │
│  SPI1 (GPIO18/19/20/21)─► CAN-FD HAT ──► can1      (CAN-FD)   │
│                                                                 │
│  SPI3 (GPIO0/1/2/3)   ──► AS5048A daisy-chain (4 SEA encoders) │
│                              /dev/spidev3.0                     │
│                                                                 │
│  UART0 GPIO14 (TX) ────────► BNO085 RX                         │
│  UART0 GPIO15 (RX) ◄──────── BNO085 TX    /dev/ttyAMA0         │
│  GPIO25           ─────────► BNO085 RESET                      │
│                                                                 │
│  GPIO4  (Pin  7) ──► left_toe_switch                           │
│  GPIO5  (Pin 29) ──► left_heel_switch                          │
│  GPIO6  (Pin 31) ──► right_toe_switch                          │
│  GPIO13 (Pin 33) ──► right_heel_switch                         │
└─────────────────────────────────────────────────────────────────┘
```

All device paths, GPIO numbers, and baud rates can be overridden in
[zeus_description/urdf/zeus_ros2_control.xacro](zeus_description/urdf/zeus_ros2_control.xacro).

> **Raspberry Pi config.txt** — SPI3 is not enabled by default. Add to `/boot/firmware/config.txt`:
> ```
> dtoverlay=spi3-1cs
> ```

---

## Current Limitations

- `zeus_gazebo` is a scaffold — simulation is not functional yet.
- `zeus_control_interface/rl_policy_node.py` is a placeholder — no RL policy loaded yet.
- Link lengths in `zeus_sensor_fusion/config/kinematics.yaml` are placeholders.
- SPI3 requires `dtoverlay=spi3-1cs` in `/boot/firmware/config.txt`.
- CAN TX from Pi to ODrive uses classic CAN (1 Mbps) not CAN-FD, due to MCP2518FD SPI driver
  instability at 5 Mbps TX under 1 kHz load. For 5 actuators on one bus, this limits TX bandwidth
  to ~65% utilisation — consider splitting 3+2 across both CAN ports.
- ODrive position commands are in revolutions (ODrive native units). Calibrate each axis in
  odrivetool before commanding motion.
