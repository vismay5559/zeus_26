# zeus_bringup

How to start the Pi side.

## Launch files

| | starts | commands the robot? |
|---|---|---|
| `robot.launch.py` | `link_node`, `robot_state_publisher`, optionally `rerun_node` | no |
| `walk.launch.py` | all of the above, plus a policy | yes, if `enable:=true` |

```bash
ros2 launch zeus_bringup robot.launch.py
ros2 launch zeus_bringup walk.launch.py                          # commands, enable false
ros2 launch zeus_bringup walk.launch.py enable:=true             # stored gait
ros2 launch zeus_bringup walk.launch.py policy:=rl enable:=true  # RL policy
```

| argument | default | |
|---|---|---|
| `port` | `auto` | device path to override discovery |
| `rerun` | `off` | `connect` or `save` |
| `rerun_host` | | laptop IP, for `connect` |
| `rerun_path` | `zeus.rrd` | use an absolute path |
| `rerun_decimate` | 10 | |
| `shm_only` | `false` | Fast DDS shared memory only (`config/fastdds_shm.xml`): less jitter, but topics invisible from other machines |
| `policy` | `passthrough` | `walk.launch.py`: `passthrough` or `rl` |
| `enable` | `false` | `walk.launch.py` |

## The `zeus` command

`scripts/zeus`, installed as `ros2 run zeus_bringup zeus`. Everything it does
needs a launch already running in another terminal; it never starts one.

| | |
|---|---|
| `zeus check` | pre-flight: board rate, every drive, IMU, spring encoders, foot switches, estimator. Exits non-zero if anything is wrong. Sends nothing |
| `zeus stop` | `/zeus/stand_down`: the drives idle, the stack keeps running |
| `zeus record NAME` | save a run (`~/zeus_runs/NAME.npz`) while the robot moves |
| `zeus tune NAME` | per-joint tracking metrics and which gain to change |
| `zeus tune --compare` | every recorded run, side by side |
| `zeus gains show` | the gains in `~/.zeus/gains.yaml`, and what the board took |
| `zeus gains push` | send them to the board (`/zeus/set_gains`) |
| `zeus gains set JOINT --vel X --pos Y --vel-int Z` | change one and send it |

`ZEUS_RUNS` and `ZEUS_GAINS` move the run directory and the gains file. The
tuning maths itself is `zeus_link.tuning`, so it is unit-tested rather than
living in a script.

## Config

| | |
|---|---|
| `config/gains.yaml` | starting drive gains, copied to `~/.zeus/gains.yaml` on first use |
| `config/link.yaml` | `link_node` parameters |
| `config/99-zeus-stm32.rules` | udev: `/dev/zeus_stm32`, `dialout` access, ModemManager kept off the port. Install steps are in the file |
| `config/fastdds_shm.xml` | shared-memory-only transport, for `shm_only:=true` |

## Stopping

`ctrl-c` sends `enable` false before closing the port. Without stopping the
launch:

```bash
ros2 service call /zeus/stand_down std_srvs/srv/Trigger
ros2 service call /zeus/resume     std_srvs/srv/Trigger
```
