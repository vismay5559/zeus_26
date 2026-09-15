# zeus_control_interface

**Where the RL policy lives.**

```
zeus_control_interface/
└── zeus_control_interface/
    ├── rl_policy_node.py                 ← write the policy here
    └── hardcoded_actuator_test_node.py   from the old ros2_control stack (see below)
```

`rl_policy_node.py` is already registered — `ros2 run zeus_control_interface
rl_policy_node` and `ros2 launch zeus_bringup walk.launch.py policy:=rl` both
run its `main()`. It is currently empty, so it needs one.

## What the policy does

Every ~4 ms:

1. read the newest `/zeus/state`
2. turn it into the observation the network was trained on
3. run the network
4. publish `/zeus/command`: a **residual** in radians per joint, and `enable`

The STM32 adds that residual to the gait it is already playing. Output zeros and
the robot walks the stored gait; output nothing and it stops.

## The observation

`zeus_link.convert.policy_block(msg)` gives the 52 values of the packet's policy
block as one `np.float32` array, in this order:

| slice | field | size | units |
|---|---|---:|---|
| 0 | `pelvis_z` | 1 | m above the stance ground |
| 1–4 | `quat` | 4 | w, x, y, z — body → world |
| 5–7 | `gyro` | 3 | rad/s, body frame |
| 8–10 | `vel_hdg` | 3 | m/s: lateral, forward, vertical |
| 11–20 | `joint_pos` | 10 | rad, output side |
| 21–30 | `joint_vel` | 10 | rad/s |
| 31–34 | `spring_angle` | 4 | rad, spring deflection |
| 35–44 | `ref_angle` | 10 | rad, the stored gait right now |
| 45–48 | `contact` | 4 | 0/1: L toe, L heel, R toe, R heel |
| 49–50 | `foot_z` | 2 | m: right, left — may be NaN |
| 51 | `phase` | 1 | 0..1 stride clock |

**Raw SI values.** The STM32 applies no scaling, clipping or normalisation — do
all of that here, exactly as in training. Replace NaNs in `foot_z` before the
network sees them.

Joint arrays follow the [joint map](../README.md#joint-map).

## Template

A complete `rl_policy_node.py` to start from. Everything except the three
marked lines is plumbing that should not need to change.

```python
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from zeus_link.convert import RESIDUAL_LIMIT_RAD, policy_block
from zeus_link.qos import COMMAND_QOS, STATE_QOS
from zeus_msgs.msg import NexusCommand, NexusState

RATE_HZ = 250.0
HANDSHAKE_TICKS = 50        # board must read IDLE/ARMED this long before enabling
STALE_SEC = 0.02            # no new state for 20 ms -> stop enabling


class RLPolicyNode(Node):

    def __init__(self):
        super().__init__('rl_policy')
        self.enable = bool(self.declare_parameter('enable', False).value)

        self.state = None
        self.state_time = 0.0       # when seq last advanced
        self.confirmed = 0
        self.handshake_done = False

        self.create_subscription(NexusState, 'zeus/state', self.on_state, STATE_QOS)
        self.pub = self.create_publisher(NexusCommand, 'zeus/command', COMMAND_QOS)
        self.create_timer(1.0 / RATE_HZ, self.step)

        self.policy = None  # <-- 1. load the network here

    def on_state(self, msg):
        if self.state is None or msg.seq != self.state.seq:
            self.state_time = time.monotonic()
        self.state = msg

    def step(self):
        cmd = NexusCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.residual_rad = [0.0] * 10
        cmd.enable = False

        s = self.state
        fresh = s is not None and (time.monotonic() - self.state_time) < STALE_SEC

        if not self.handshake_done:
            # enable false until the link node is listening AND the board says
            # IDLE or ARMED - which is what clears a latched fault
            ready = (fresh and self.pub.get_subscription_count() > 0
                     and s.safety_state in (NexusState.SAFETY_IDLE, NexusState.SAFETY_ARMED))
            self.confirmed = self.confirmed + 1 if ready else 0
            self.handshake_done = self.confirmed >= HANDSHAKE_TICKS
        elif fresh:
            obs = policy_block(s)                                  # 52 raw SI values
            obs = np.nan_to_num(obs)                              # <-- 2. your normalisation
            residual = np.zeros(10, dtype=np.float32)             # <-- 3. self.policy(obs)
            residual = np.clip(residual, -RESIDUAL_LIMIT_RAD, RESIDUAL_LIMIT_RAD)
            cmd.residual_rad = residual.astype(np.float32)
            cmd.enable = self.enable

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RLPolicyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

Notes on the choices in it:

- **The handshake.** The STM32 latches every fault and refuses to arm until it
  sees `enable` false. The template keeps sending it until the board *reports*
  IDLE or ARMED, so restarting the node is always enough to recover. Do not
  shorten it to a fixed count from startup: that finishes before DDS discovery
  has connected the node, the frames are dropped, and the board stays latched.
  This was found against `fake_board` in `gait_passthrough_node`, which had
  exactly that bug.
- **Clipping here is deliberate** — unlike in `zeus_link`, which passes values
  through. This is where the policy's authority is decided; the board rejects
  anything past ±0.628 rad outright.
- **Staleness.** If `/zeus/state` stops updating, the template keeps publishing
  but with `enable` false, so the board stands its drives down instead of acting
  on an old observation. It re-enables by itself when state resumes.
- **Types.** `residual_rad` must be a `float32` array or a list of Python floats
  — see [zeus_msgs](../zeus_msgs/README.md).
- **Heavy inference.** If the network takes more than a few ms on the Pi, run it
  in a separate thread and let `step()` publish the latest result, so the
  command stream stays at 250 Hz.

Test it without moving anything: `walk.launch.py policy:=rl` (enable defaults
to false) and watch `ros2 topic echo /zeus/command` or the residual panel in
Rerun.

## hardcoded_actuator_test_node.py

Left as it was. It publishes to `/single_actuator_command_controller/commands`,
a ros2_control topic that no longer exists, so it does nothing on this stack.
The equivalent now is `zeus_link`'s `gait_passthrough_node`.
