# zeus_description

The robot model, generated from the Fusion 360 export. **Do not edit
[urdf/zeus.urdf](urdf/zeus.urdf) by hand.**

```
urdf/zeus_raw.urdf          exactly as Fusion 360 exported it (+ meshes/)
config/zeus_model.yaml      everything the export does not know or gets inconvenient
        │
        └─ scripts/clean_urdf.py ─► urdf/zeus.urdf      what ROS, Pinocchio and the STM32 generator read
```

## What the clean-up does

- **`base_link`** is a new root at the hip centre (where the roll and pitch axes
  of the two hips meet, averaged) with the robot's own axes: **X forward,
  Y left, Z up**. The export's axes were Fusion's world: +X left, −Y forward.
- **Names.** Links and the 14 moving joints get real names (`left_knee_motor`,
  `left_knee_pitch`, …) instead of `Revolute-77`. The 10 actuated joints use the
  joint map's names, the same ones `/joint_states` carries.
- **Axes.** Every pitch joint turns about +Y and every roll joint about +X, on
  both legs, so a positive hip pitch swings either foot backwards and a positive
  hip roll moves either foot left. The export had them mirrored.
- **Springs.** Hip pitch and knee are two joints on one axis: the motor output
  (`left_knee_pitch`), then the spring's deflection (`left_knee_pitch_spring`).
  The leg's real angle is their sum.
- **Frames** Fusion has no reason to export: `imu_link` and the four contact
  points `left_toe`, `left_heel`, `right_toe`, `right_heel`.

## Changing the model

```bash
# new export from Fusion: replace urdf/zeus_raw.urdf and meshes/, then
python3 zeus_description/scripts/clean_urdf.py            # writes urdf/zeus.urdf
python3 zeus_description/scripts/clean_urdf.py --suggest  # IMU/toe/heel positions measured from the meshes
python3 -m pytest zeus_description/test -q                # CI runs this too
```

The script stops with a message if the export has a link or moving joint the
YAML does not name, or if a joint declared `pitch` does not turn about the
robot's Y axis. CI fails if `zeus.urdf` is not what the script makes, so a hand
edit or a forgotten regeneration cannot go unnoticed.

Frame positions in the YAML are in the **robot's** frame at the zero pose (hip
centre, X forward), so they can be checked with a ruler on the real robot.

```bash
ros2 launch zeus_description display.launch.py gui:=true    # laptop: sliders + RViz
```

## Not right yet

| | now | TODO |
|---|---|---|
| design | Fusion version with **hip roll as the parent** | the real robot has hip pitch first: re-export |
| joint limits | ±π, springs ±0.2 rad | mechanical stops, spring travel |
| `imu_link` | centre of the electronics box's top face, axes = robot axes | the chip's real position and how it is rotated |
| toe / heel | front and rear edge of each sole, mid-width | where each switch actually touches down |
| spring order | motor joint first, spring second on each axis | confirm against the CAD |
| `/joint_states` | carries the 10 motor joints only | the 4 spring joints need the encoder→joint order from the firmware, until then TF stops at each spring |
