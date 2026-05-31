# Zeus — Bipedal Robot ROS 2 Workspace

Zeus is a 10-DOF bipedal robot controlled from a Raspberry Pi running ROS 2.
This workspace contains every package needed to go from raw sensor hardware to
a full state estimate that a walking controller can consume.

---

## What the Robot Has

| Hardware | Details |
|----------|---------|
| **Actuators** | 10 × ODrive S1 brushless motor drivers, 5 per CAN bus (`can0` / `can1`) |
| **CAN-FD HAT** | Waveshare dual-channel CAN-FD HAT in Mode A — 5 Mbps data phase, hijacks SPI0 + SPI1 |
| **After-spring encoders** | 10 × AS5048A 13-bit magnetic encoders, SPI3 daisy-chain on `/dev/spidev3.0` |
| **IMU** | Adafruit BNO085, SHTP protocol over hardware UART `/dev/ttyAMA0` at 3 Mbaud |
| **Contact switches** | 4 × mechanical switches wired directly to Raspberry Pi GPIO pins |
| **CAN buses** | 2 × SocketCAN (`can0`, `can1`), CAN-FD at 5 Mbps data rate |

### Leg and Joint Mapping

```
Joint index │ Actuator         │ CAN bus │ ODrive node ID
────────────┼──────────────────┼─────────┼───────────────
  joint_0   │ left_hip_pitch   │  can0   │  1
  joint_1   │ left_hip_roll    │  can0   │  2
  joint_2   │ left_knee_pitch  │  can0   │  3
  joint_3   │ left_ankle_pitch │  can0   │  4
  joint_4   │ waist_pitch      │  can0   │  5
  joint_5   │ right_hip_pitch  │  can1   │  1
  joint_6   │ right_hip_roll   │  can1   │  2
  joint_7   │ right_knee_pitch │  can1   │  3
  joint_8   │ right_ankle_pitch│  can1   │  4
  joint_9   │ waist_roll       │  can1   │  5
```

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
  ├── BNO085 IMU (SPI) ─────────────────┤──► zeus_hardware_interface
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

## Interface Count: 44 state + 10 command

### Command Interfaces (10)

```
joint_0/target_actuator_angle
joint_1/target_actuator_angle
...
joint_9/target_actuator_angle
```

### State Interfaces (44)

```
After-spring encoder angle (10):
  joint_0/after_spring_angle … joint_9/after_spring_angle

ODrive load encoder position (10):
  joint_0/load_encoder_position … joint_9/load_encoder_position

ODrive torque estimate (10):
  joint_0/torque_estimate … joint_9/torque_estimate

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
controller_manager loop:    1000 Hz
Actuator CAN-FD commands:   1000 Hz   (5 Mbps data phase, near-zero bus contention)
AS5048A encoder reads:      1000 Hz   (SPI3 burst, 20 bytes per cycle)
GPIO contact switch reads:  1000 Hz   (pre-opened sysfs fd — no open() overhead)
ODrive CAN telemetry:       1000 Hz
BNO085 IMU UART stream:      400 Hz   (3 Mbaud, FIFO-buffered, no clock stretching)
Sensor fusion publish:       100 Hz
```

---

## Package Layout

```
nexus_final_ws/
├── zeus/                       Metapackage (groups all other packages)
├── zeus_can_interface/         Low-level SocketCAN driver for ODrive CAN messages
├── zeus_hardware_interface/    ROS 2 control hardware plugin — the main hardware bridge
├── zeus_description/           Robot URDF/xacro and ros2_control configuration
├── zeus_bringup/               Launch files and controller YAML config
├── zeus_control_interface/     Python placeholder for RL policy commands
├── zeus_sensor_fusion/         Contact-aided InEKF state estimator
└── zeus_gazebo/                Simulation scaffold (not yet active)
```

---

## Package Details

### `zeus_can_interface`

A thin C++ library wrapping Linux SocketCAN.

- Opens a raw CAN socket on any named interface (`can0`, `can1`, ...).
- `send_position_target(node_id, position)` — sends ODrive `Set_Input_Pos` command.
- `read_frame(frame)` — non-blocking single-frame read.
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
- [zeus_hardware_interface/src/sensor_utils.cpp](zeus_hardware_interface/src/sensor_utils.cpp) — BNO085 SHTP/SPI reads

#### Lifecycle

| Stage | What happens |
|-------|-------------|
| `on_init()` | Reads xacro hardware params; allocates all state/command arrays; detects sensor elements from URDF |
| `on_configure()` | Opens CAN sockets, encoder SPI, IMU SPI; exports GPIO pins via sysfs and pre-opens value file descriptors |
| `on_activate()` | Marks hardware ready; enables ODrive cyclic telemetry |
| `read()` | 1. Reads AS5048A encoders → `after_spring_angle`. 2. Drains ODrive CAN frames → `load_encoder_position`, `torque_estimate`. 3. Reads BNO085 SHTP packets → IMU states. 4. Reads GPIO switches → `contact` states |
| `write()` | Sends `target_actuator_angle` to each ODrive via `Set_Input_Pos` |
| `on_deactivate()` | Stops issuing commands |
| `on_cleanup()` | Closes all fds; unexports GPIO pins; closes SPI/CAN sockets |

#### AS5048A Encoder Chain

10 encoders daisy-chained on **SPI3** (`/dev/spidev3.0`). SPI3 is used because the CAN-FD HAT in Mode A physically owns SPI0 (can0) and SPI1 (can1) for its 5 Mbps data link. SPI3 must be enabled by adding `dtoverlay=spi3-1cs` to `/boot/firmware/config.txt` on the Raspberry Pi.

Each encoder returns a 13-bit angle (0–8191 counts = 0–2π radians). The driver reads all 10 in one 20-byte SPI burst, applies parity checking, and runs a small first-order low-pass filter (α = 0.2) to reduce noise.

SPI3 wiring:

```
MOSI → GPIO2  (Pin 3)
MISO → GPIO1  (Pin 28)
SCLK → GPIO3  (Pin 5)
CS0  → GPIO0  (Pin 27)   master chip-select for the daisy-chain
```

#### BNO085 IMU

The BNO085 is driven over **hardware UART** rather than SPI. SPI0 and SPI1 are occupied by the CAN-FD HAT, so the IMU is moved to the dedicated hardware UART peripheral on GPIO14/15, which feeds directly into the RP1's hardware FIFO with zero clock-stretching.

The SHTP (Sensor Hub Transport Protocol) packet framing is identical to the SPI variant. The driver opens `/dev/ttyAMA0` at 3 Mbaud in raw mode (`cfmakeraw`), streams SHTP packets from the UART RX FIFO, and accumulates partial packets in an internal buffer between `read()` calls.

Three report types are enabled at startup:
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

No INT pin is required on UART — the host reads whatever bytes are available each control cycle.

#### Mechanical Contact Switches

The 4 switches are wired directly to GPIO pins. The driver uses the Linux sysfs GPIO interface (`/sys/class/gpio/`):

1. **`on_configure()`**: exports each pin, sets direction to `in`, opens `/sys/class/gpio/gpioN/value` and keeps the file descriptor open.
2. **`read()`**: `lseek(fd, 0, SEEK_SET); read(fd, buf, 1)` — reads a single `'0'` or `'1'` byte without reopening, giving ~1 µs latency.
3. **`on_cleanup()`**: closes fds, unexports pins.

Each switch has an `active_low` parameter. If `active_low=true`, a LOW pin signal means the switch is pressed.

---

### `zeus_description`

URDF and ros2_control xacro files.

Key files:
- [zeus_description/urdf/zeus_urdf.xacro](zeus_description/urdf/zeus_urdf.xacro) — robot geometry
- [zeus_description/urdf/zeus_ros2_control.xacro](zeus_description/urdf/zeus_ros2_control.xacro) — hardware plugin config, all sensor/joint declarations

The xacro declares:
- Hardware plugin: `zeus_hardware_interface/ZeusSystemHardware`
- All 10 joint command/state interfaces
- IMU sensor element
- 4 mechanical switch sensor elements (with gpio_pin and active_low params)
- ODrive telemetry interfaces

---

### `zeus_bringup`

Launch files and controller YAML.

#### Controller config: [zeus_bringup/config/zeus_controllers.yaml](zeus_bringup/config/zeus_controllers.yaml)

| Controller | Type | Purpose |
|-----------|------|---------|
| `joint_state_broadcaster` | JointStateBroadcaster | Publishes joint encoder/ODrive states |
| `imu_sensor_broadcaster` | IMUSensorBroadcaster | Publishes `/imu_sensor_broadcaster/imu` |
| `contact_state_broadcaster` | JointStateBroadcaster | Publishes switch contact states |
| `rl_forward_command_controller` | ForwardCommandController | Accepts 10-joint target angle array |

#### Launch files

| File | Purpose |
|------|---------|
| [hardware.launch.py](zeus_bringup/launch/hardware.launch.py) | Full robot stack — all sensors + actuators + all controllers |
| [single_actuator_command_test.launch.py](zeus_bringup/launch/single_actuator_command_test.launch.py) | Command-only test for one joint (no sensors needed) |
| [hardcoded_single_actuator_test.launch.py](zeus_bringup/launch/hardcoded_single_actuator_test.launch.py) | Automatically sends a small command sequence to one joint |
| [sim.launch.py](zeus_bringup/launch/sim.launch.py) | Simulation bringup (Gazebo — not yet active) |

---

### `zeus_sensor_fusion`

A full state estimator for the lower torso. Given IMU data, encoder angles, and contact switch readings, it outputs: velocity, position, orientation, and height above ground — all with uncertainty estimates.

Key files:
- [zeus_sensor_fusion/zeus_sensor_fusion/lie_group.py](zeus_sensor_fusion/zeus_sensor_fusion/lie_group.py) — SE_{N+2}(3) Lie group math
- [zeus_sensor_fusion/zeus_sensor_fusion/kinematics.py](zeus_sensor_fusion/zeus_sensor_fusion/kinematics.py) — Zeus leg forward kinematics
- [zeus_sensor_fusion/zeus_sensor_fusion/inekf.py](zeus_sensor_fusion/zeus_sensor_fusion/inekf.py) — the filter itself
- [zeus_sensor_fusion/zeus_sensor_fusion/sensor_fusion_node.py](zeus_sensor_fusion/zeus_sensor_fusion/sensor_fusion_node.py) — ROS 2 node
- [zeus_sensor_fusion/config/kinematics.yaml](zeus_sensor_fusion/config/kinematics.yaml) — link lengths **(must be measured)**
- [zeus_sensor_fusion/config/inekf_params.yaml](zeus_sensor_fusion/config/inekf_params.yaml) — filter noise parameters

#### The Algorithm: Contact-Aided Right-Invariant EKF (InEKF)

Based on Hartley et al. 2019, "Contact-Aided Invariant Extended Kalman Filtering for Robot State Estimation."

**Why not a standard EKF?**

A standard EKF linearises the system around the current estimate. When the state is far from the truth (e.g. at startup), the linearisation error causes the filter to diverge or converge slowly. The InEKF exploits the geometric structure of the robot's motion: the error dynamics on the SE_{N+2}(3) Lie group are *independent of the state estimate*, which makes the filter far more robust and fast-converging.

**State representation**

The filter tracks a matrix `X` on the Lie group SE_{N+2}(3), plus a bias vector `θ`:

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

The covariance `P` tracks uncertainty in all of the above.

**Predict step (runs at IMU rate ~400 Hz)**

Uses the gyroscope and accelerometer to propagate `X` forward in time. The IMU biases are subtracted first. The integration uses exact SO(3) exponential maps (Γ₀, Γ₁, Γ₂) rather than a first-order approximation, so it stays accurate even at large angular rates.

**Measurement update (runs when a foot is in contact)**

Each time new encoder angles arrive (1000 Hz), forward kinematics computes where the foot contact point is in the body frame. Since the foot is on the ground and not sliding (contact assumption), this gives a constraint:

```
world-frame contact position = R · (FK foot position) + p
```

This constraint is applied as a right-invariant observation update. The observation noise is propagated through the FK Jacobian, so encoder angle uncertainty is correctly accounted for.

**Contact management**

- **Rising edge** (switch pressed for N consecutive reads): the foot's world position is initialised from the current state estimate plus FK. A new column is added to `X` and the covariance is augmented.
- **Falling edge** (switch released): the foot's column is removed from `X` and marginalised out of `P`.
- Debounce is configurable (`contact_debounce_count`, default 3 reads).

**Forward kinematics chain (per leg)**

```
T_hip_offset
  → Ry(hip_pitch)
  → Rx(hip_roll)
  → translate(0, 0, -thigh_length)
  → Ry(knee_pitch)
  → translate(0, 0, -shank_length)
  → Ry(ankle_pitch)
  → translate(0, 0, -foot_height)
  ──► B_p_BC  (contact point in body frame)
```

Waist joints (joint_4, joint_9) are NOT included — they connect lower torso to upper body, and since the IMU lives in the lower torso, the waist angles do not affect foot position in the body frame.

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/zeus/estimated_height` | `std_msgs/Float64` | z-coordinate of IMU in world frame (metres above ground) |
| `/zeus/estimated_velocity` | `geometry_msgs/TwistWithCovariance` | Body-frame linear velocity + 3×3 covariance |
| `/zeus/estimated_pose` | `geometry_msgs/PoseWithCovariance` | Full position + orientation + covariance |
| `/zeus/contact_state` | `std_msgs/Bool` | True if any foot is currently in confirmed contact |
| `/zeus/imu_bias_gyro` | `geometry_msgs/Vector3Stamped` | Estimated gyroscope bias (rad/s) |
| `/zeus/imu_bias_accel` | `geometry_msgs/Vector3Stamped` | Estimated accelerometer bias (m/s²) |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | BNO085 IMU data — drives predict step |
| `/dynamic_joint_states` | `control_msgs/DynamicJointState` | Encoder angles — drives FK measurements |
| `/contact_state_broadcaster/dynamic_joint_states` | `control_msgs/DynamicJointState` | Switch states — drives contact management |

#### Height Definition

Height = `X[2, 4]` = the z-component of the IMU world-frame position. When a foot first makes contact the filter initialises that foot's world position assuming the ground is at z = 0. As the robot moves, the IMU z-coordinate tracks the true height above that ground plane.

---

### `zeus_control_interface`

Python package for sending commands to the robot.

- [rl_policy_node.py](zeus_control_interface/zeus_control_interface/rl_policy_node.py) — placeholder for a reinforcement learning policy that publishes joint targets to `/rl_forward_command_controller/commands`.
- [hardcoded_actuator_test_node.py](zeus_control_interface/zeus_control_interface/hardcoded_actuator_test_node.py) — publishes a simple command sequence for hardware bring-up.

---

### `zeus_gazebo`

Simulation scaffold. Not yet functional — Gazebo world, model, and launch files are placeholders.

---

## Build

Full build from workspace root:

```bash
cd ~/nexus_final_ws
colcon build
source install/setup.bash
```

Faster incremental build (hardware + bringup only):

```bash
colcon build --packages-select zeus_can_interface zeus_hardware_interface zeus_description zeus_bringup
source install/setup.bash
```

Sensor fusion only:

```bash
colcon build --packages-select zeus_sensor_fusion
source install/setup.bash
```

---

## Running on Hardware

### 1. Bring up CAN buses

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up
```

### 2. Launch the full hardware stack

```bash
source install/setup.bash
ros2 launch zeus_bringup hardware.launch.py
```

### 3. Launch sensor fusion (in a second terminal)

```bash
source install/setup.bash
ros2 launch zeus_sensor_fusion sensor_fusion.launch.py
```

### 4. Check everything started correctly

```bash
# List hardware interfaces (expect 44 state + 10 command)
ros2 control list_hardware_interfaces

# List active controllers
ros2 control list_controllers

# Watch estimated height
ros2 topic echo /zeus/estimated_height

# Watch estimated velocity
ros2 topic echo /zeus/estimated_velocity
```

---

## Sending Actuator Commands

The full robot forward command controller expects 10 values (one per joint, in radians):

```bash
ros2 topic pub /rl_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

To move only `joint_0` (left hip pitch) by a small amount:

```bash
ros2 topic pub /rl_forward_command_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" --once
```

> **Use very small values during initial bring-up.** The command is sent directly to the ODrive position controller. The units depend on your ODrive gear ratio configuration.

---

## Single-Actuator Tests

These tests bypass all sensors and only test the command path:

```
ROS 2 command → hardware interface → SocketCAN → ODrive
```

Useful for checking that a single motor responds before the full sensor stack is wired.

**Command-only test (you control the target value):**

```bash
ros2 launch zeus_bringup single_actuator_command_test.launch.py can_interface:=can0 node_id:=1
ros2 topic pub /single_actuator_command_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.02]}" --once
```

**Hardcoded sequence (moves automatically then returns to zero):**

```bash
ros2 launch zeus_bringup hardcoded_single_actuator_test.launch.py can_interface:=can0 node_id:=1 target:=0.02
```

Monitor CAN traffic while these run:

```bash
candump can0
```

---

## Before You Trust the State Estimates

The kinematics config at [zeus_sensor_fusion/config/kinematics.yaml](zeus_sensor_fusion/config/kinematics.yaml) contains **placeholder link lengths**:

```yaml
thigh_length: 0.30   # PLACEHOLDER — measure hip joint to knee joint (metres)
shank_length: 0.30   # PLACEHOLDER — measure knee joint to ankle joint (metres)
foot_height:  0.05   # PLACEHOLDER — measure ankle joint to contact point (metres)
left_hip_offset:  [0.0,  0.05, 0.0]   # PLACEHOLDER — lateral hip position in body frame
right_hip_offset: [0.0, -0.05, 0.0]   # PLACEHOLDER
```

Measure the actual distances on the physical robot and update this file before relying on height or velocity estimates. The filter will run with placeholder values but the estimates will be wrong.

---

## Useful Debug Commands

```bash
# Hardware
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 control list_controllers

# Sensors
ros2 topic echo /imu_sensor_broadcaster/imu
ros2 topic echo /dynamic_joint_states
ros2 topic echo /contact_state_broadcaster/dynamic_joint_states

# State estimates
ros2 topic echo /zeus/estimated_height
ros2 topic echo /zeus/estimated_velocity
ros2 topic echo /zeus/estimated_pose
ros2 topic echo /zeus/imu_bias_gyro
ros2 topic echo /zeus/imu_bias_accel

# CAN bus
candump can0
candump can1

# Check GPIO switch reading
# Press a switch physically, then:
cat /sys/class/gpio/gpio4/value     # left_toe
cat /sys/class/gpio/gpio5/value     # left_heel
cat /sys/class/gpio/gpio6/value     # right_toe
cat /sys/class/gpio/gpio13/value    # right_heel

# Check IMU UART stream
stty -F /dev/ttyAMA0 3000000 raw
hexdump -C /dev/ttyAMA0 | head -20    # should show SHTP packets at 400 Hz

# Verify SPI3 encoder device exists (requires dtoverlay=spi3-1cs in config.txt)
ls /dev/spidev3.0
```

---

## Hardware Wiring Summary

```
┌─────────────────────────────────────────────────────────────────┐
│                      Raspberry Pi 5                             │
│                                                                 │
│  SPI0 (GPIO8/9/10/11) ──► CAN-FD HAT ──► can0  (5 Mbps CAN-FD)│
│  SPI1 (GPIO18/19/20/21)─► CAN-FD HAT ──► can1  (5 Mbps CAN-FD)│
│                                                                 │
│  SPI3 (GPIO0/1/2/3)   ──► AS5048A daisy-chain (10 encoders)   │
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

All device paths, GPIO numbers, and baud rates can be overridden in [zeus_description/urdf/zeus_ros2_control.xacro](zeus_description/urdf/zeus_ros2_control.xacro).

> **Raspberry Pi config.txt** — SPI3 is not enabled by default. Add this line to `/boot/firmware/config.txt` and reboot:
> ```
> dtoverlay=spi3-1cs
> ```

---

## Current Limitations

- `zeus_gazebo` is a scaffold — simulation is not functional yet.
- `zeus_control_interface/rl_policy_node.py` is a placeholder — no RL policy is loaded yet.
- Link lengths in `zeus_sensor_fusion/config/kinematics.yaml` are placeholders and must be measured on the physical robot.
- SPI3 is not enabled by default on the Raspberry Pi — add `dtoverlay=spi3-1cs` to `/boot/firmware/config.txt` before the encoder chain will appear as `/dev/spidev3.0`.
- The BNO085 UART driver has not been tested on the final hardware; verify `/dev/ttyAMA0` maps to GPIO14/15 on your specific Pi 5 image (it should, but some images reassign ttyAMA0 to Bluetooth).
- ODrive position commands are in the ODrive's native units (revolutions, depending on encoder CPR and gear ratio configuration on the ODrive). Calibrate each axis before commanding motion.
