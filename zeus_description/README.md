# zeus_description

The robot model: [urdf/zeus.urdf.xacro](urdf/zeus.urdf.xacro).

- **Joint names** are the joint map — the same ten names `/joint_states`
  carries — so `robot_state_publisher` turns the STM32's joint angles straight
  into TF.
- **The leg chain** is the one the STM32 estimator's forward kinematics uses:
  `hip_pitch (Y) → hip_roll (X) → thigh → knee_pitch (Y) → shank → ankle_pitch (Y) → foot`.
- **`base_link` is the pelvis and the IMU frame**, since the estimator treats
  the IMU as the body.

```bash
ros2 launch zeus_description display.launch.py gui:=true    # laptop: sliders + RViz
xacro urdf/zeus.urdf.xacro > zeus.urdf                      # plain URDF, e.g. for Rerun
```

## Not measured yet

| | value | source |
|---|---|---|
| thigh, shank | 0.30 m | placeholder, same as STM32 `robot_config.h` |
| foot height | 0.05 m | placeholder |
| hip offset | ±0.05 m in Y | placeholder |
| waist | 0.08 m above the pelvis, roll then pitch | assumption |
| limits | ±π | wide on purpose; the STM32 enforces the real envelope |

Link lengths are kept equal to `robot_config.h` so this model and the
estimator agree with each other. When the robot is measured, **change both**,
then set `ROBOT_CONFIG_CALIBRATED` on the STM32.

Meshes from the CAD can replace the boxes and rods in each `<visual>` without
touching the joints.
