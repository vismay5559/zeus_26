# zeus_link

The Raspberry Pi's end of the USB link to the STM32. **The only package that
opens the serial port** — everything else goes through its topics.

```
STM32 ──USB, 444 B @ 1 kHz──► link_node ──► /zeus/state    zeus_msgs/NexusState
                                        ──► /joint_states  sensor_msgs/JointState
STM32 ◄──USB, 52 B @ 250 Hz── link_node ◄── /zeus/command  zeus_msgs/NexusCommand
```

## Executables

| | ROS? | what |
|---|---|---|
| `link_node` | yes | Owns the port. Publishes state, forwards commands, logs link health and every change in safety state, estimator state, health bits and ODrive errors |
| `gait_passthrough_node` | yes | Zero residual at 250 Hz. `enable:=false` (default) holds the board in IDLE; `enable:=true` walks the stored gait |
| `link_check` | no | Reads the link and prints rate, loss, source (`ROBOT` or `LEG TEST`) and status once a second. First thing to run on a new Pi |
| `fake_board` | no | A fake STM32 on a pty (Linux/macOS): 1 kHz packets, commands parsed, the firmware's arm/fault/re-arm rules. Run the whole stack with no robot |

```bash
ros2 run zeus_link link_check --joints
ros2 run zeus_link link_node --ros-args -p port:=/dev/ttyACM1
ros2 run zeus_link gait_passthrough_node --ros-args -p enable:=true
```

Usually started by `zeus_bringup` instead.

## link_node

**Parameters** (defaults in `zeus_bringup/config/link.yaml`)

| | default | |
|---|---|---|
| `port` | `auto` | `/dev/zeus_stm32` if the udev rule is installed, else the one device with USB IDs `0483:5740`. The Nucleo's ST-LINK port is `0483:374x` and is never picked |
| `joint_state_rate_hz` | 100 | 0 disables `/joint_states` |
| `stats_period_sec` | 5 | Link health log interval |
| `frame_id` | `base_link` | |

**Services**

| | |
|---|---|
| `/zeus/stand_down` | Latch: every command is forwarded with `enable` false, and one is sent immediately. The board idles its drives but the link stays healthy, so it does not fault |
| `/zeus/resume` | Release the latch |

**What it logs**, so a bring-up session reads like a story:

```
[zeus_link]: link open on /dev/zeus_stm32; publishing zeus/state, joint_states at 100 Hz
[zeus_link]: safety state -> BOOT
[zeus_link]: health faults: LINK
[zeus_link]: health: all watched subsystems OK
[zeus_link]: safety state -> IDLE
[zeus_link]: left_knee_pitch: axis CLOSED_LOOP_CONTROL
[zeus_link]: safety state -> ARMED
[zeus_link]: right_ankle_pitch: ODrive error 0x08000200 DC_BUS_UNDER_VOLTAGE | BRAKE_RESISTOR_DISARMED
[zeus_link]: rx 1000.0 Hz | published  998.7 Hz | commands 250.0 Hz | 51234 pkts, 0.000% lost (0), 0 junk B
```

`rx` is what arrived from the board; `published` is what went out on
`/zeus/state`. If `published` is lower, the Pi is too busy to publish every
packet and is skipping — the topic still carries the newest one.

**Command path.** `residual_rad` is converted to turns (÷ 2π) and sent with a
link-owned sequence number. A command with the wrong length or a NaN is dropped
with an error. A residual beyond the STM32's ±0.628 rad is **still sent** — the
board is the single authority on limits — but logged as a warning, since the
board will reject it.

**On shutdown** it sends `enable` false three times before closing the port,
so the drives idle immediately rather than 200 ms later via a link fault.

## Python modules

Usable from a policy, a notebook or a test, with or without ROS.

| module | |
|---|---|
| `nexus_proto` | Packet layouts, CRC, `NexusState.parse`, `NexusCommand.pack`, `JOINT_NAMES`, all constants. **Copied from stm32_zeuss `pi/`** — see below |
| `nexus_link` | `NexusLink`: a thread that drains the port at 1 kHz; `latest()` for control, `drain()` for logging. **Copied** |
| `convert` | `fill_state_msg`, `residual_rad_to_turns`, `policy_block(state)`, `joint_state_arrays` |
| `ports` | `find_port("auto")` |
| `qos` | `STATE_QOS`, `COMMAND_QOS` — use these |
| `odrive_names` | `error_names(0x200)` → `['DC_BUS_UNDER_VOLTAGE']`, `axis_state_name(8)` |
| `synthetic` | `state_packet(seq=…)`: a CRC-valid packet, for testing without a robot |

```python
from zeus_link.convert import policy_block
obs = policy_block(msg)          # np.float32, 52 values, packet order, raw SI
```

## Keeping the protocol in step

`nexus_proto.py` and `nexus_link.py` are copies; each file's header names the
firmware commit it came from. The firmware repo holds the originals and checks
them against the C header. **When the protocol version changes, copy both files
over** — do not edit them here. A version mismatch is not silent: every packet
fails the version check, `link_check` shows 0 Hz and junk bytes climbing.

## Tests

No ROS needed:

```bash
cd zeus_link && python3 -m pytest test -q
```

- the copied protocol still matches firmware v6: sizes, CRC, round trip,
  corrupt and wrong-version packets rejected, framing across junk and split reads
- `NexusState.msg` matches `convert.STATE_FIELDS`, the parser's dataclass and
  the wire format, field for field, including every constant and the joint map
- `fill_state_msg` against a stand-in that enforces rclpy's type assertions
  (a Python `int` into a `float32`, a `float64` array, a wrong length all fail);
  under `colcon test` it also runs against the real generated class
- port discovery picks the link over the ST-LINK
- the reader thread, end to end through a serial loopback

End to end, with a built and sourced workspace (CI runs this too):

```bash
python3 zeus_link/test/e2e_fake_board.py
```

It runs `link_node` and `gait_passthrough_node` against `fake_board` and
checks: 1 kHz on `/zeus/state`, every command intact and in sequence, arming,
stand-down, and — the two bugs it was written after — that `/zeus/state` keeps
publishing when the board faults, and that a policy restarted after a fault
re-arms it. Against the code before those fixes, exactly those checks fail.
