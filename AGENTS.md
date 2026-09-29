# Zeus — orientation for an agent picking this up cold

Read this before touching anything. It is the map, the build commands, and the
list of things that have already cost days. It is deliberately specific: if a
fact here contradicts the code, the code wins and this file is a bug.

Zeus is a bipedal robot. Two repositories, one machine each:

| repo | runs on | what it is |
|---|---|---|
| **`stm32_zeuss`** | STM32H7S3L8H6 (NUCLEO-H7S3L8, board MB1737) | firmware: the 1 kHz control loop, the state estimator, CAN to the drives |
| **`zeus_26`** | Raspberry Pi 5, hostname `nexus`, Ubuntu 24.04, ROS 2 Jazzy | the ROS workspace: the link node, URDF, visualisation, CLI |

They are joined by **one USB cable** and a versioned binary protocol. The board
does the fast, safety-critical work. The Pi does the thinking. If the Pi goes
quiet the board copes on its own.

Both repos push **directly to `main`**. Do not create branches.

---

## The hardware, concretely

- **8 actuated joints**, 4 per leg: hip pitch, hip roll, knee pitch, ankle
  pitch. There is no waist — it is bolted. Driven by **ODrive S1** over
  **CAN-FD**, two buses (FDCAN1 = left leg, FDCAN2 = right).
- **Joint index = `bus * 4 + (node - 1)`.** This mapping is the single source of
  truth for every per-joint array in both packets. It is written down in
  `link_proto.h` as `NEXUS_J_*` and in `nexus_proto.py` as `JOINT_NAMES`, and
  `tools/check_proto.py` fails if they disagree.
- **Series-elastic** hip pitch and knee pitch: the drive reports the motor side,
  an **AS5047P** encoder reports the spring's deflection, and the real joint
  angle is their sum. Two SPI daisy chains, one per leg, one chip-select each.
- **BNO085 IMU**, on the torso. The estimator uses its raw accel and gyro; its
  own fused quaternion comes across too, but only for comparison.
- **2 foot switches**, one per foot, at the **centre of the sole**. There were
  two per foot (toe and heel) until protocol v9.

## The link

USB CDC. The board is the device, the Pi is the host.

- USB ID **`0483:5740`** ("STM32 Virtual ComPort"). `0483:374B`/`374E` is the
  **ST-LINK**, a different chip — it is the debugger and text console, not the
  data link.
- udev gives it the stable name **`/dev/zeus_stm32`**
  (`zeus_bringup/config/99-zeus-stm32.rules`).
- **Protocol v9.** State packet **426 bytes, board → Pi, every 1 ms**. Command
  packet 44 bytes, Pi → board, ~250 Hz. Gains message 106 bytes.
- The state packet opens with a contiguous **44-float32 policy block** so the RL
  policy can take one array with no per-field parsing
  (`zeus_link.convert.policy_block`).
- Commands are a **residual**, not a target: `drive target = ref_angle +
  residual`. Zeros with `enable: true` walks the stored gait unaided.

**The version byte is load-bearing.** A board and a workspace at different
protocol versions refuse to talk and say so, rather than silently misreading
bytes. Flash and deploy together.

---

## Repo map

### `zeus_26` (7 packages)

| package | what it does |
|---|---|
| `zeus_msgs` | interface definitions only: `NexusState.msg`, `NexusCommand.msg`, `SetGains.srv`. No code. |
| `zeus_link` | the heart. `nexus_proto.py` (wire format), `nexus_link.py` (threaded reader), `link_node.py` (ROS node), `link_check`, `link_validate`, `fake_board`, `tuning.py` |
| `zeus_bringup` | launch files, the `zeus` CLI (`scripts/zeus`), udev rule, config |
| `zeus_description` | URDF generated from a Fusion 360 export by `scripts/clean_urdf.py` + `config/zeus_model.yaml`. **Edit the YAML, never the URDF.** |
| `zeus_control_interface` | ros2_control hardware interface |
| `zeus_rerun` | Rerun visualisation |
| `zeus` | metapackage |

### `stm32_zeuss`

Two CubeMX projects: **`Boot/`** (internal flash, sets up XSPI2 and jumps) and
**`Appli/`** (executes in place from external flash at `0x70000000`).

Everything worth reading is in **`Appli/App/`**:

| file | what it does |
|---|---|
| `link_proto.h` | **the protocol.** Both packet structs, every constant, byte offsets |
| `link_usb.c/.h` | the USB link, framing, CRC, and `link_usb_diag()` |
| `app.c` | the 1 kHz robot loop |
| `fusion.c`, `inekf.c`, `lie_group.c` | contact-aided right-invariant EKF (Hartley et al. 2019) |
| `zeus_kinematics.c` + `_model.h` | forward kinematics + exact Jacobian. **`_model.h` is GENERATED** |
| `contact.c` | foot switch debounce |
| `act_odrive.c`, `enc_as5047p.c`, `imu_bno085.c` | drivers |
| `safety.c`, `health.c`, `watchdog.c` | what is allowed to move, and why not |
| `console.c` | non-blocking printf — see the gotchas |
| `nexus_mode.h` | **which program the firmware is.** `NEXUS_MODE_ROBOT`, `_LEG_CAN`, `_LEG_TORQUE`, `_IMU` |
| `test_leg_can.c` | the single-leg bench test. **The user leaves uncommitted edits here** |

---

## Commands

### Workspace

```bash
cd ~/zeus_26
colcon build --symlink-install && source install/setup.bash
```

Tests, **the way CI runs them** — per directory, from the repo root:

```bash
python3 -m pytest zeus_link/test -q          # the big one
python3 -m pytest zeus_rerun/test -q         # skips without rerun-sdk; exit 5 is normal
python3 -m pytest zeus_description/test -q
colcon test && colcon test-result --verbose
```

### Firmware

```bash
cd ~/Downloads/stm32_zeuss
for d in /opt/st/stm32cubeide_*/plugins/com.st.stm32cube.ide.mcu.externaltools.{gnu-tools-for-stm32,ninja,cmake,cubeprogrammer}.*/tools/bin; do PATH="$d:$PATH"; done; export PATH

bash tools/hosttest/run.sh          # 14 suites, pure host C, no hardware
python3 tools/check_proto.py        # C struct vs Python, field by field
cmake --build Appli/build

ST_LOADER=$(ls /opt/st/stm32cubeide_*/plugins/*cubeprogrammer*/tools/bin/ExternalLoader/MX25UW25645G_NUCLEO-H7S3L8.stldr)
STM32_Programmer_CLI -c port=SWD mode=UR -el "$ST_LOADER" -d Appli/build/nexus_first_Appli.hex -v -rst
```

Only the Appli needs the external loader. Boot goes to internal flash and rarely
changes.

**Console:** the ST-LINK virtual COM port, 115200 8N1.

```bash
python3 -m serial.tools.miniterm /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3*-if01 115200
```

### Checking the link

```bash
ros2 run zeus_link link_check                 # a live one-line-per-second view
ros2 run zeus_link link_validate --seconds 60 # measures, and says which side is wrong
```

`link_validate` separates three things `link_check` conflates: the **board's**
loop rate (from its own tick counter), **delivery** (the only figure about the
cable and host), and **jitter** against the 250 Hz policy period. Exit 0 only on
PASS.

---

## Things that will waste your time

These are all real, all cost hours or days, and all have a comment in the code
explaining them. Do not undo them.

**USB**
- `USBREGEN` must be **cleared**, not set. On MB1737 `VDD33USB` is supplied from
  the board's own 3V3 rail, so the internal regulator must be off. CubeMX and
  every ST example set it. Symptom: `PWR_CSR2.USB33RDY` stays 0 forever, the
  transceiver has no power, and the host sees nothing at all — indistinguishable
  from a dead cable.
- The OTG core's **DMA is off on purpose**. D-cache is on and the control
  endpoint buffers are in cached RAM, so the core DMAs each SETUP packet into a
  line the CPU then reads stale. Symptom: device attaches, then
  `device descriptor read/64, error -32`.
- `link_usb_diag()` prints the supply rail, the control bits and the device
  state **on the ST-LINK console** — readable exactly when the host can tell you
  nothing. Use it before suspecting cables.
- The Pi 5's **USB-C is power input only.** It cannot host. Use a USB-A socket.

**The 1 kHz loop**
- **Never `printf` synchronously from the control loop.** The console UART is
  polled at 87 µs per character; a 350-byte status line cost 30 ticks a second.
  `console.c` makes printf asynchronous — characters go in a ring and the USART3
  interrupt drains them. A burst that outruns the wire is **dropped**, counted
  by `console_dropped()`.
- Any buffer a peripheral writes into must be in the non-cacheable region —
  `NEXUS_DMA_BUFFER` in `dma_buffer.h`, which must agree with **MPU region 2 in
  the Boot project**. `app.c` checks that agreement at startup.
- **XSPI2 must not be re-initialised in Appli.** The code executes in place
  through it; re-initialising tears down the mapping and the next instruction
  fetch hangs the bus forever, with no fault and no message.

**Serial**
- **One reader per port.** Two processes on one tty do not each get a copy —
  every byte goes to whoever read first, and both see a shredded stream. The
  port opens `exclusive=True` so the second one fails with a clear message.

**Generated files — do not hand-edit**
- `Appli/App/zeus_kinematics_model.h` and `tools/hosttest/zeus_kinematics_ref.h`
  come from `tools/gen_kinematics.py`. CI checks the URDF's sha256 embedded in
  the model header against `zeus_26`'s actual URDF.
- `zeus_description/urdf/zeus.urdf` comes from `clean_urdf.py` + the YAML.
- **Pinocchio's current wheels are broken** (eigenpy built against a mismatched
  numpy ABI). `gen_kinematics.py --model-only` regenerates the firmware tables
  with numpy alone; only the host test's reference needs Pinocchio. The walk
  simulation (`tools/sim/run.sh --check`) also needs it, so it may only be
  runnable in CI.

**CubeMX**
- It regenerates `main.c`, `main.h` and the MSP files. Changes outside
  `USER CODE` blocks will be lost. Several deliberate deviations are documented
  in place — read the comments before "fixing" something that looks wrong.
- Pin names are CubeMX's. `L_TOE`/`R_TOE` are now the **single switch on each
  foot**; `L_HEEL`/`R_HEEL` are unconnected. Renaming needs the .ioc, which is a
  GUI action.

---

## How this codebase is written

Match it. It is unusually heavily commented and that is deliberate.

- **Comments explain _why_, and what went wrong.** Many name the specific bug
  they prevent. Keep that when you edit around them; delete one only if it has
  become false.
- **Tests state their verdict first.** Cases are hand-built with the expected
  answer written down before the maths runs, and the comment says what a failure
  would mean on the real robot. A metric that is quietly wrong sends someone to
  tune the wrong gain on a real leg.
- **Protocol changes bump `NEXUS_PROTO_VERSION`** and must be made in
  `link_proto.h`, `stm32_zeuss/pi/nexus_proto.py` (**canonical**), the copy in
  `zeus_26/zeus_link/zeus_link/nexus_proto.py`, and `NexusState.msg` together.
  `check_proto.py` and `test_msg_matches_proto.py` enforce it.
- **Stage only your own changes.** The user keeps bench edits uncommitted in
  `test_leg_can.c`.
- **Verify the way CI does** — grep the whole tree after a rename, run tests from
  the repo root, and check CI with `gh run list` after pushing.

---

## Where things stand

Working:
- The USB link: 1000 Hz at the board, 100% delivery, `/zeus/state` at ~999.98 Hz.
- Host test suites on both sides, and the walk simulation in CI.

Open problems:
- **CAN is dead.** The bus scan finds nothing: `TEC=0 REC=0`, all drives silent.
  `TEC=0` is the informative part — the board never attempted a transmission
  that failed, so it is not detecting a bus fault; there is nothing there to
  answer. Drives unpowered, CAN H/L not connected, or termination missing.
- **Hip load encoder reads garbage**, which confounds gain tuning.
- **Foot switch debounce is unmeasured.** `CONTACT_MAKE_TICKS 3` /
  `CONTACT_BREAK_TICKS 8` (3 ms / 8 ms) are plausible defaults, not measured
  against these switches. Both are overridable at configure time.
- `s_tick_pending` in `test_leg_can.c` is a flag, not a counter, so missed ticks
  are invisible to the loop.
- rerun-sdk is not installed on the Pi. **Never `pip install --user` there** —
  it needs numpy 2; use a venv with `--system-site-packages`.

## Safety

- The robot moves only while commands keep arriving. 200 ms of silence and the
  link fault idles every axis. After a fault, one message with `enable: false`
  is required before `enable: true` is accepted again.
- Gains are **refused while armed** — a velocity gain changing under load is a
  step change in torque with a leg's weight behind it.
- **Grounding:** the Pi and the board on separate mains supplies are bonded
  through the USB cable. Keep both on one power strip. Once motor power is live,
  put a USB isolator between them — note the link currently enumerates at high
  speed, and most cheap isolators are full-speed only.
