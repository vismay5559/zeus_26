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

## Config

| | |
|---|---|
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
