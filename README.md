# Zeus — Raspberry Pi workspace

Zeus is a 10-DOF biped. This ROS 2 Jazzy workspace is everything that runs on its
Raspberry Pi — which, by design, is not very much.

**The STM32 does the robot. The Pi does the policy.**

```
                ┌──────────────────────────── STM32H7 (stm32_zeuss) ───────────────────────────┐
 BNO085 IMU ────┤                                                                              │
 4 spring enc ──┤  1 kHz loop: sensors → state estimator → stored gait → safety → 10 ODrives   │
 4 foot switch ─┤                                                                              │
 CAN × 2 ───────┤                                                                              │
                └──────────────┬───────────────────────────────────────────────▲───────────────┘
                   state, 444 B│ every 1 ms                    residual, 52 B  │ ~250 Hz
                        USB OTG HS                                             │
                ┌──────────────▼───────────────────── Raspberry Pi ────────────┴───────────────┐
                │ zeus_link   link_node ──► /zeus/state ──► your RL policy ──► /zeus/command    │
                │                       └─► /joint_states ─► robot_state_publisher (TF)        │
                │ zeus_rerun  rerun_node ◄── /zeus/state, /zeus/command ──► Wi-Fi ──► laptop    │
                └──────────────────────────────────────────────────────────────────────────────┘
```

The STM32 owns all real-time sensing, the state estimate, the reference gait and
every safety decision. It streams the finished state to the Pi 1000 times a
second. The policy on the Pi answers with a **residual** — a small correction —
and the STM32 drives every joint to:

```
drive target = stored gait (ref_angle) + residual
```

So a policy that outputs zeros walks the stored gait, and a policy that stops
answering stops the robot. Nothing on the Pi touches a CAN bus, a GPIO pin or a
sensor.

- [The contract with the STM32](#the-contract-with-the-stm32)
- [Joint map](#joint-map)
- [Packages](#packages)
- [Setting up the Pi](#setting-up-the-pi)
- [Running](#running)
- [Topics and services](#topics-and-services)
- [Watching it in Rerun](#watching-it-in-rerun)
- [Where the RL policy goes](#where-the-rl-policy-goes)
- [Troubleshooting](#troubleshooting)
- [Before the full robot walks](#before-the-full-robot-walks)

---

## The contract with the STM32

The firmware lives in [stm32_zeuss](https://github.com/vismay5559/stm32_zeuss). Its
`Appli/App/link_proto.h` is the definition of both packets; `zeus_link` carries a
byte-exact Python copy, and the firmware repo's `tools/check_proto.py` verifies
that copy against the C header.

| | STM32 → Pi | Pi → STM32 |
|---|---|---|
| **what** | state: estimate, joints, reference, sensors, health | residual per joint + enable flag |
| **size** | 444 bytes | 52 bytes |
| **rate** | 1000 Hz | ~250 Hz (the STM32 interpolates between commands) |
| **units** | SI, radians on the output shaft | turns on the wire, **radians in ROS** |

What the board does with commands — worth knowing before anything moves:

1. **It starts in `BOOT`** and leaves only once every watched subsystem is
   healthy: IMU, spring encoders, both CAN buses, loop timing, **and the link —
   meaning commands have to be arriving.** Launching the link alone leaves the
   board in `BOOT` with a `LINK` health fault; that is expected.
2. **Commands with `enable` false keep it in `IDLE`**: link healthy, drives idle.
3. **The first command with `enable` true arms it.** Drives go to closed loop
   and the state reads `ARMED`.
4. **Every command is checked**: finite, |residual| ≤ **0.1 turn (0.628 rad)**,
   target inside the joint envelope, and no jump bigger than 0.5 turn.
5. **If commands stop for 200 ms**, the link fault idles every axis → `FAULT`.
6. **After any fault it will not re-arm** until it has seen a command with
   `enable` false. `gait_passthrough_node` does this handshake on start, and
   keeps `enable` false until the board actually reports IDLE; your policy
   should too (the template does). A fixed-length handshake from startup is
   not enough — it can finish before DDS has connected the policy to the link
   node, and every frame of it is lost.
7. **Each `enable` false starts a new command session.** Within a session the
   sequence number must advance; across one it may start again. That is what
   lets a restarted `link_node`, counting from 0, re-arm a board that has been
   running for hours. (Firmware before the `s_seq_valid` fix in stm32_zeuss
   refused it until the STM32 was rebooted.)

## Joint map

Every per-joint array — `joint_pos`, `joint_vel`, `ref_angle`, `act_*`,
`residual_rad` — uses this order. `index = bus × 5 + (node − 1)`.

| index | bus | node | joint |
|---:|---:|---:|---|
| 0 | 0 | 1 | left_hip_pitch |
| 1 | 0 | 2 | left_hip_roll |
| 2 | 0 | 3 | left_knee_pitch |
| 3 | 0 | 4 | left_ankle_pitch |
| 4 | 0 | 5 | waist_roll |
| 5 | 1 | 1 | right_hip_pitch |
| 6 | 1 | 2 | right_hip_roll |
| 7 | 1 | 3 | right_knee_pitch |
| 8 | 1 | 4 | right_ankle_pitch |
| 9 | 1 | 5 | waist_pitch |

In code: `zeus_link.nexus_proto.JOINT_NAMES`, `JOINT_INDEX["right_knee_pitch"]`,
or the `J_*` constants on `zeus_msgs/NexusState`. The URDF and `/joint_states`
use the same names. The waist has no stored gait; its `ref_angle` reads 0.

---

## Packages

```
zeus_26/
├── zeus/                    metapackage
├── zeus_msgs/               NexusState.msg, NexusCommand.msg
├── zeus_link/               the USB link: protocol, reader, link_node, passthrough, link_check
├── zeus_control_interface/  ← YOUR RL POLICY goes here (rl_policy_node.py)
├── zeus_rerun/              live plots and 3D in Rerun, from ROS or straight off serial
├── zeus_description/        URDF from the Fusion 360 export: joints, springs, IMU, toe/heel frames
└── zeus_bringup/            robot.launch.py, walk.launch.py, link config, udev rule
```

| package | type | what it is for |
|---|---|---|
| [zeus_msgs](zeus_msgs/README.md) | ament_cmake | The two messages, field-for-field with the USB packets |
| [zeus_link](zeus_link/README.md) | ament_python | The only thing that opens the serial port |
| [zeus_control_interface](zeus_control_interface/README.md) | ament_python | The RL policy |
| [zeus_rerun](zeus_rerun/README.md) | ament_python | Visualisation on your laptop |
| [zeus_description](zeus_description/README.md) | ament_cmake | Robot model for TF, RViz and Rerun |
| [zeus_bringup](zeus_bringup/README.md) | ament_cmake | How to start it all |

### What was removed, and why

The previous workspace had the Pi drive the hardware itself: SocketCAN to the
ODrives through a CAN-FD HAT, SPI to the spring encoders, UART to the IMU, GPIO
to the foot switches, and a Python estimator. All of that moved to the STM32.
The code is in git history (commit `1863bf1` and before) if it is ever needed.

| removed | was | now |
|---|---|---|
| `zeus_can_interface` | SocketCAN driver for the ODrives | STM32 `act_odrive.c`, both CAN buses |
| `zeus_hardware_interface` | ros2_control plugin for CAN, SPI, UART, GPIO | STM32 drivers; `zeus_link` on the Pi |
| `zeus_sensor_fusion` | contact-aided InEKF in Python | STM32 `inekf.c` — a port of that code |
| `zeus_gazebo` | empty simulation scaffold | nothing yet |
| `zeus_bringup` scripts | ODrive diag, backlash test, joint commander over raw CAN | `link_check`; the STM32 leg test |

---

## Setting up the Pi

Once per Pi. Ubuntu 24.04 with ROS 2 Jazzy (`ros-jazzy-ros-base` is enough).

```bash
# 1. the workspace
mkdir -p ~/zeus_ws && cd ~/zeus_ws
git clone https://github.com/vismay5559/zeus_26.git src
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# 2. a stable name for the board, and keep ModemManager off it
sudo cp src/zeus_bringup/config/99-zeus-stm32.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER            # log out and back in after this

# 3. Rerun SDK, in its own venv. It needs numpy 2; ROS Jazzy's Python is built
#    on numpy 1.26, and a --user install breaks apt's scipy and friends.
python3 -m venv --system-site-packages ~/rerun_venv
~/rerun_venv/bin/pip install rerun-sdk

# 4. build
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Add `source /opt/ros/jazzy/setup.bash && source ~/zeus_ws/install/setup.bash`
to `~/.bashrc` so every terminal has it.

**The STM32 must be flashed with `NEXUS_MODE_ROBOT`** (`Appli/App/nexus_mode.h`).
The bench test modes enumerate the same USB device but never send a state
packet, which from here looks exactly like a dead link.

---

## Running

Step by step, the first time. Each step proves the one before it.

```bash
# 1. Is the link alive? No ROS; sends nothing to the board.
ros2 run zeus_link link_check --joints
#    want: ~1000 Hz, 0.000% lost, joints reporting

# 2. The link as ROS topics. Nothing is commanded; the board stays in BOOT.
ros2 launch zeus_bringup robot.launch.py
ros2 topic hz /zeus/state                  # another terminal

# 3. Commands flowing, enable false: the board reaches IDLE, nothing moves.
ros2 launch zeus_bringup walk.launch.py

# 4. Walk the stored gait, no policy.
ros2 launch zeus_bringup walk.launch.py enable:=true

# 5. Your policy.
ros2 launch zeus_bringup walk.launch.py policy:=rl enable:=true
```

Add `rerun:=connect rerun_host:=<laptop IP>` to any launch to watch it live —
after `source ~/rerun_venv/bin/activate` in that terminal.

**Stopping.** `ctrl-c` on the launch sends `enable` false on the way out. To
stop moving *without* stopping anything:

```bash
ros2 service call /zeus/stand_down std_srvs/srv/Trigger    # idle the drives
ros2 service call /zeus/resume     std_srvs/srv/Trigger    # hand enable back
```

### Without the robot

`fake_board` stands in for the STM32 on a pseudo-terminal: 1 kHz packets out,
commands parsed, the firmware's arm/fault/re-arm rules modelled. Everything on
the Pi runs for real, which makes it the place to develop the policy.

```bash
ros2 run zeus_link fake_board                                            # terminal 1
ros2 launch zeus_bringup walk.launch.py port:=/tmp/zeus_fake_board enable:=true   # terminal 2
```

It does not simulate the robot — `joint_pos` does not respond to the residual.

---

## Testing the USB cable, and watching the leg test live

You can test the STM32 → Pi link on the bench, without the robot and without
ROS, in three steps. Each step proves the one before it.

### The two USB ports on the Nucleo

| port | for |
|---|---|
| **ST-LINK** | flashing, and the text console (115200) |
| **USB user port** | **this link** — USB OTG HS, 480 Mbit/s. Plug it into the Pi |

The board is a USB device and the Pi is the host. On the Pi it becomes a serial
port, `/dev/ttyACM*`, USB ID `0483:5740`, carrying binary packets: 444 bytes of
state every millisecond out, commands in.

**Which firmware sends packets:** `NEXUS_MODE_ROBOT` always, and
`NEXUS_MODE_LEG_CAN` — the single-leg test streams its run as the same packet
(stm32_zeuss `LEGTEST_USB_STREAM`, on by default). In the leg test the board
**ignores commands**, so nothing on the Pi can move the leg. The bench leg is on
CAN bus 0, so it shows up as the **left** leg: `left_hip_pitch`,
`left_knee_pitch`, `left_ankle_pitch`, with `ref_angle` = the target it sent.

### 1. The cable enumerates

```bash
lsusb | grep 0483:5740       # STMicroelectronics Virtual COM Port
lsusb -t                     # that device must say 480M
sudo dmesg | tail            # "new high-speed USB device", "cdc_acm ... ttyACMn"
```

Not listed: wrong port (ST-LINK), a charge-only cable, or the firmware is not
running. `12M` / "full-speed": swap the cable or port — 12 Mbit/s cannot carry
the stream.

### 2. Packets arrive intact — no ROS needed

```bash
cd zeus_26
PYTHONPATH=zeus_link python3 -m zeus_link.link_check --joints
```

Healthy: **~1000 Hz, 0.000% lost, 0 junk B**, and a `LEG TEST` or `ROBOT` column.
0 Hz means the board is in a mode that sends nothing; junk bytes climbing
usually means ModemManager has the port — install the udev rule.

### 3. Watch it live on your laptop

```bash
# laptop, once
python3 -m venv --system-site-packages ~/rerun_venv && ~/rerun_venv/bin/pip install rerun-sdk
# laptop, each time
~/rerun_venv/bin/rerun          # viewer opens and waits
hostname -I                     # note the laptop's IP

# Pi (same venv set up once)
cd zeus_26
PYTHONPATH=zeus_rerun:zeus_link ~/rerun_venv/bin/python -m zeus_rerun.rerun_serial \
    --connect <LAPTOP_IP> --degrees
```

Run the leg test: the **Joints** tab plots each joint's actual angle against its
target, in degrees, live. Same rerun-sdk version on both machines; same network.

With ROS on the Pi, the same view goes through `link_node` and `/zeus/state`:

```bash
source ~/rerun_venv/bin/activate
ros2 launch zeus_bringup robot.launch.py rerun:=connect rerun_host:=<LAPTOP_IP> rerun_degrees:=true
```

**No Pi yet?** Plug the user port straight into the laptop, run step 2 there,
and use `rerun_serial --spawn --degrees`.

---

## Topics and services

| name | type | direction | rate | QoS |
|---|---|---|---|---|
| `/zeus/state` | `zeus_msgs/NexusState` | link_node → | up to 1 kHz | best effort, depth 1 |
| `/zeus/command` | `zeus_msgs/NexusCommand` | → link_node | ~250 Hz | reliable, depth 1 |
| `/joint_states` | `sensor_msgs/JointState` | link_node → | 100 Hz | default |
| `/tf`, `/robot_description` | | robot_state_publisher → | | |
| `/zeus/stand_down` | `std_srvs/Trigger` | service | | |
| `/zeus/resume` | `std_srvs/Trigger` | service | | |

**Always use `zeus_link.qos.STATE_QOS` and `COMMAND_QOS`** when subscribing or
publishing. A reliable subscriber on the best-effort state topic receives
nothing and reports nothing.

`/zeus/state` is **latest-wins**: if a subscriber or the publisher falls behind,
old packets are skipped rather than queued. `seq` advances once per STM32 tick,
so a gap in `seq` is exactly how many were skipped. Field-by-field meaning is in
[NexusState.msg](zeus_msgs/msg/NexusState.msg).

---

## Watching it in Rerun

Rerun is split into an SDK (logs data) and a Viewer (the window), and they do
not need to be on the same machine. **The Pi logs; your laptop looks.** The
viewer is never in the control path — losing Wi-Fi costs plots, never a command.

```bash
# laptop - same rerun-sdk version as the Pi (venv on Ubuntu, plain pip elsewhere)
rerun                                    # opens and waits on port 9876

# Pi - alongside the robot
source ~/rerun_venv/bin/activate
ros2 launch zeus_bringup walk.launch.py rerun:=connect rerun_host:=192.168.1.50
```

Or record on the Pi and look later — best for anything fast:

```bash
ros2 launch zeus_bringup walk.launch.py rerun:=save rerun_path:=/home/$USER/run1.rrd
scp pi@<pi>:run1.rrd . && rerun run1.rrd          # laptop
```

Or with no ROS at all — the STM32 plugged into your laptop:

```bash
python3 -m zeus_rerun.rerun_serial --spawn
```

The viewer opens with four tabs: every joint's **actual vs reference**, the
estimator and sensors, the policy's residual with torques and health, and a 3D
pelvis. Details: [zeus_rerun](zeus_rerun/README.md).

---

## Where the RL policy goes

**`zeus_control_interface/zeus_control_interface/rl_policy_node.py`** — the file
exists, is empty, and is already wired to `ros2 run zeus_control_interface
rl_policy_node` and to `walk.launch.py policy:=rl`. It needs a `main()`.

The shape of it: subscribe `/zeus/state`, build the observation with
`zeus_link.convert.policy_block(msg)` (the 52 raw values, in order), run the
network, publish `residual_rad` on `/zeus/command`. A complete template is in
[zeus_control_interface/README.md](zeus_control_interface/README.md); the
working minimal example is `zeus_link/gait_passthrough_node.py`.

---

## Troubleshooting

| symptom | cause |
|---|---|
| `no STM32 USB link found` | Cable in the ST-LINK port instead of the USB user port; or the Appli is not running |
| port opens, `no packets` | Board flashed with a test mode — set `NEXUS_MODE_ROBOT` and reflash |
| `Permission denied: /dev/ttyACM0` | Not in `dialout` (and logged back in) |
| `could not open port … busy` | Another program holds it: `link_check`, `rerun_serial`, a second `link_node` |
| rate well under 1000 Hz | Pi overloaded, or the tty low-latency mode did not take; run `link_check` alone |
| board stays `BOOT`, `LINK` fault | Nothing is publishing `/zeus/command` — expected with `robot.launch.py` alone |
| `enable:=true` but never `ARMED` | The policy logs why every 2 s: no link node, no state, or the board's health faults |
| `FAULT` after stopping the policy | Expected: commands stopped. Restart; the handshake clears it |
| subscriber gets nothing | QoS mismatch — use `zeus_link.qos` |
| Rerun connects, shows nothing | Viewer not running, port 9876 blocked, or SDK and viewer versions differ |
| `rerun-sdk is not importable` | The venv is not active in that terminal |
| scipy / numpy errors after installing rerun | It went into user site. `python3 -m pip uninstall --break-system-packages rerun-sdk pyarrow numpy`, then use the venv |

---

## Before the full robot walks

This workspace is ready. These are the open items it depends on, all on the
STM32 or the robot itself:

- **Geometry and joint signs are unmeasured.** Link lengths, hip offsets, each
  joint's sign and zero (`robot_config.h`). Until measured and
  `ROBOT_CONFIG_CALIBRATED` is set, the estimator reports `CONVERGING`, never
  `OK` — so `fused_valid` stays 1 and height/velocity should not be trusted.
- **The command envelope is a placeholder** (±10 turns, 0.5-turn step).
- **Robot mode does not configure the drives.** Gains, control mode and zeros
  must already be saved on all ten ODrives.
- **CAN bus budget.** Robot mode sends 5 position frames per bus every
  millisecond on classic CAN; with 1 kHz encoder telemetry from each drive that
  exceeds one bus. Telemetry rates or command rate have to come down first.
