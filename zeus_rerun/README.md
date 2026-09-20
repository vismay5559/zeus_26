# zeus_rerun

Live plots and 3D of the robot in [Rerun](https://rerun.io).

## Where things run

Rerun is two pieces that do not have to share a machine:

```
STM32 ──USB──► Raspberry Pi ──Wi-Fi / Ethernet──► your laptop
               zeus_rerun (the SDK)               Rerun Viewer (the window)
               logs, headless                     where you look
```

The Pi has no screen and does not need one. **The viewer is never in the
control path**: a dropped connection is a blank plot, never a stalled leg.

## Install

**Pi, or any Ubuntu machine with ROS** — in its own venv:

```bash
python3 -m venv --system-site-packages ~/rerun_venv
~/rerun_venv/bin/pip install rerun-sdk
```

and activate it in any terminal that runs `rerun_node`:

```bash
source ~/rerun_venv/bin/activate
ros2 launch zeus_bringup walk.launch.py rerun:=connect rerun_host:=192.168.1.50
```

**Do not `pip install --user --break-system-packages rerun-sdk`.** Every
rerun-sdk with the APIs used here (0.23 onwards) requires numpy 2, and pip will
install it into your user site, where it shadows the numpy 1.26 that Ubuntu
24.04 and ROS Jazzy are built against. Tested on Ubuntu 24.04: rclpy and our
messages happened to survive, but apt's scipy broke immediately (`numpy.dtype
size changed`), and so does any apt package compiled against numpy 1.

Why the venv works: `--system-site-packages` keeps rclpy and the ROS packages
visible, while numpy 2 and rerun stay inside the venv. `rerun_node` is installed
with `#!/usr/bin/env python3` (see `setup.cfg`), so it runs on the venv's Python
when the venv is active. Every other node keeps `/usr/bin/python3` and the
system numpy, venv or not.

**Laptop** — the viewer. On Ubuntu use a venv the same way and run
`~/rerun_venv/bin/rerun`. On Windows or macOS, `pip install rerun-sdk` is fine.

**Use the same rerun-sdk version everywhere** (`pip show rerun-sdk`). The SDK
and viewer refuse to talk across versions. Tested with 0.37.2.

## Three ways to use it

### 1. Live, Pi → laptop

```bash
# laptop - start this first
rerun

# Pi - as part of a launch
ros2 launch zeus_bringup walk.launch.py rerun:=connect rerun_host:=192.168.1.50

# ...or on its own, next to an already running robot
ros2 run zeus_rerun rerun_node --ros-args -p mode:=connect -p host:=192.168.1.50
```

Port 9876, plain TCP. Find the laptop's IP with `ipconfig` (Windows) or
`ip addr` (Linux), and allow Python/rerun through its firewall.

### 2. Record now, look later — start here

```bash
# Pi
ros2 launch zeus_bringup walk.launch.py rerun:=save rerun_path:=/home/$USER/run1.rrd

# laptop
scp <user>@<pi>:run1.rrd .
rerun run1.rrd
```

No network during the run, and recordings scrub: the 40 ms where something went
wrong can be found and stepped through. **Give an absolute path** — a relative
one lands in whatever directory `ros2 launch` was started from. `.rrd` files
are git-ignored.

### 3. No ROS, no Pi — STM32 on your laptop's USB

```bash
cd zeus_26
PYTHONPATH=zeus_rerun:zeus_link python3 -m zeus_rerun.rerun_serial --spawn
# Windows PowerShell:
#   $env:PYTHONPATH="zeus_rerun;zeus_link"; python -m zeus_rerun.rerun_serial --spawn
```

Needs `rerun-sdk pyserial numpy` (in a venv on Linux). Also works on the Pi as
`ros2 run zeus_rerun rerun_serial --connect <laptop>` — but only when
`link_node` is not running, since only one program can hold the port.

## What you see

The viewer opens with this layout (rearrange freely):

| tab | panels |
|---|---|
| **Joints: actual vs reference** | one plot per joint, 8 in a grid: **actual** (`joint_pos`), **commanded** (`act_target` - what the drive was told, after the residual, the safety envelope and the interpolator) and **reference** (the stored gait alone). Commanded against actual is the pair that matters when tuning |
| **tracking** | commanded − actual, per joint, in one plot |
| **Estimator & sensors** | pelvis height, heading-frame velocity, IMU gyro and accel, foot switches, foot heights, spring deflection, gait phase |
| **Policy, drives & health** | the residual the policy sent (ROS mode), torque, joint velocity, safety state, health bits, loop time, overruns |
| **3D** | the pelvis pose from the fused quaternion, and the feet at their estimated height |

Everything is on the **`robot` timeline** — seconds since the STM32 booted,
from its tick counter. Recordings made through ROS and straight off serial line
up the same way.

The 3D view shows **height, not stride**: the packet carries foot height but
not foot x/y, so the feet hang under the hips. A full model from the URDF with
animated joints is the natural next step.

## Parameters

`rerun_node`, or the matching `rerun_*` launch arguments:

| | default | |
|---|---|---|
| `mode` | `save` | `connect`, `save` or `spawn` |
| `host` | | viewer's IP, for `connect` |
| `port` | 9876 | viewer's port |
| `path` | `zeus.rrd` | file, for `save` |
| `decimate` | 10 | log every Nth state: 10 = 100 Hz of plots |
| `degrees` | false | joint angles and velocities in degrees (`--degrees` for `rerun_serial`, `rerun_degrees:=true` for the launch) |

Decimation is about the Pi, not the viewer: each logged state is ~20 log calls.
100 Hz is comfortable on a Pi 4. Set `decimate:=1` on a laptop, or on the Pi for
a short recording of something fast like a foot strike.

## Files

| | |
|---|---|
| `zeus_rerun/viz.py` | all the logging and the layout; works on a ROS message or a parsed packet |
| `zeus_rerun/rerun_node.py` | ROS: subscribes `/zeus/state` and `/zeus/command` |
| `zeus_rerun/rerun_serial.py` | no ROS: reads the port itself |
| `test/test_viz.py` | logs synthetic packets and commands to a real `.rrd` |
