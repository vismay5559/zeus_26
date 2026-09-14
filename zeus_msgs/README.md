# zeus_msgs

The two messages on the link.

| message | topic | mirrors |
|---|---|---|
| [NexusState](msg/NexusState.msg) | `/zeus/state` | the STM32's 444-byte state packet, field for field |
| [NexusCommand](msg/NexusCommand.msg) | `/zeus/command` | the 52-byte command, in radians instead of turns |

Field names in `NexusState` are identical to `zeus_link.nexus_proto.NexusState`,
so code written against one works on the other — `zeus_link.convert.policy_block`
and `zeus_rerun` both rely on that. `zeus_link`'s tests fail if they drift.

```bash
ros2 interface show zeus_msgs/msg/NexusState
ros2 topic echo /zeus/state --field safety_state
```

## Units and conventions

- Angles: **radians on the output shaft**, including `residual_rad`. The wire
  protocol carries the residual in turns; `zeus_link` converts.
- Every `[10]` array is in joint-map order; the `J_*` constants name the indices.
- `header.stamp` is Pi receive time. `seq` is the STM32 tick — use it for timing.
- `foot_z` is `NaN` for a foot whose kinematics are invalid (`fk_valid` bit clear).
  Never 0.0, which would read as "on the ground".
- `spring_angle` is spring **deflection**, not a joint angle.

## Filling a command from Python

rclpy's generated classes are strict about types:

```python
cmd = NexusCommand()
cmd.residual_rad = residual.astype(np.float32)   # numpy float32, exactly 10
cmd.residual_rad = [0.0] * 10                    # or a list of Python floats
cmd.enable = True
```

A `float64` numpy array, or a list containing `int`s, raises an `AssertionError`.
