"""
Logging the robot's state to Rerun - shared by the ROS node and the serial tool.

Works on anything with NexusState's field names: a zeus_msgs/NexusState from
ROS, or a zeus_link.nexus_proto.NexusState straight off the serial port. The
two share names on purpose, so there is one logging path, not two.

WHAT IS LOGGED, AND WHERE (rerun groups plots by path)

  joints/<joint>        actual (joint_pos) vs reference (ref_angle), rad -
                        or degrees with degrees=True, like the leg test's plots
  policy/residual       residual_rad the policy sent, every joint      (ROS only)
  velocity, torque      every joint
  estimator/*           pelvis_z, vel_hdg, fused_valid
  imu/gyro, imu/accel   raw BNO085
  contacts, feet        foot switches, foot heights
  springs               spring deflection
  gait/phase            stride clock
  status/*              safety_state, health bits, loop time, overruns
  world/pelvis          3D pose from the fused quaternion

Every value sits on the "robot" timeline, seconds since the STM32 booted
(seq / 1000). That is the board's own clock, so a recording made through ROS
and one made straight off the serial port line up the same way.

COST ON A PI

Each group is ONE log call carrying several series, not one call per value -
about 20 calls per logged state. At the default decimation of 10 (100 Hz of
plotted data) that is ~2000 calls a second, comfortably inside one Pi 4 core.
Decimate less on a laptop.
"""

from __future__ import annotations

import math
from typing import Optional, Sequence

try:
    import rerun as rr
    import rerun.blueprint as rrb
except ImportError as exc:        # pragma: no cover - message for the robot
    raise ImportError(
        "rerun-sdk is not importable. It lives in its own venv, because it needs\n"
        "numpy 2 and ROS Jazzy's Python is built against numpy 1.26:\n"
        "    python3 -m venv --system-site-packages ~/rerun_venv\n"
        "    ~/rerun_venv/bin/pip install rerun-sdk\n"
        "then `source ~/rerun_venv/bin/activate` before ros2 run / ros2 launch.\n"
        "Install the SAME rerun-sdk version wherever the viewer runs."
    ) from exc

from zeus_link.nexus_proto import CONTACT_NAMES, JOINT_NAMES

TIMELINE = "robot"
VIEWER_PORT = 9876

LEFT = [80, 140, 220]
RIGHT = [220, 120, 60]
WAIST = [170, 110, 200]
GREY = [150, 150, 150]
AXES = [[230, 80, 80], [80, 200, 90], [80, 130, 230]]

# Hip offset from robot_config.h, used only to place the feet sideways in 3D.
HIP_OFFSET_Y = 0.05


def _joint_colour(name: str):
    return LEFT if name.startswith("left") else RIGHT if name.startswith("right") else WAIST


def blueprint() -> "rrb.Blueprint":
    """The layout the viewer opens with. Every panel can be rearranged after."""
    joints = rrb.Grid(
        *[rrb.TimeSeriesView(origin=f"joints/{n}", name=n) for n in JOINT_NAMES],
        grid_columns=5,
        name="Joints: actual vs reference",
    )
    estimator = rrb.Grid(
        rrb.TimeSeriesView(origin="estimator", name="estimator"),
        rrb.TimeSeriesView(origin="imu", name="IMU"),
        rrb.TimeSeriesView(origin="contacts", name="foot switches"),
        rrb.TimeSeriesView(origin="feet", name="foot height"),
        rrb.TimeSeriesView(origin="springs", name="spring deflection"),
        rrb.TimeSeriesView(origin="gait", name="phase"),
        name="Estimator & sensors",
    )
    drives = rrb.Grid(
        rrb.TimeSeriesView(origin="policy", name="policy residual"),
        rrb.TimeSeriesView(origin="torque", name="torque"),
        rrb.TimeSeriesView(origin="velocity", name="velocity"),
        rrb.TimeSeriesView(origin="status", name="status"),
        name="Policy, drives & health",
    )
    return rrb.Blueprint(
        rrb.Tabs(joints, estimator, drives, rrb.Spatial3DView(origin="world", name="3D")),
        collapse_panels=True,
    )


def open_sink(mode: str, host: str = "", port: int = VIEWER_PORT,
              path: str = "zeus.rrd", app_id: str = "zeus") -> str:
    """
    Start a recording and point it somewhere. Returns a line saying where.

      connect   stream to a viewer already running on `host`   (Pi -> laptop)
      save      write an .rrd file to open later               (no network)
      spawn     open a viewer on THIS machine                  (needs a display)
    """
    rr.init(app_id)
    bp = blueprint()

    if mode == "connect":
        if not host:
            raise ValueError("mode 'connect' needs the viewer's host (the laptop's IP)")
        url = f"rerun+http://{host}:{port}/proxy"
        rr.connect_grpc(url, default_blueprint=bp)
        where = f"streaming to the viewer at {url}"
    elif mode == "save":
        rr.save(path, default_blueprint=bp)
        where = f"recording to {path}"
    elif mode == "spawn":
        rr.spawn(port=port, default_blueprint=bp)
        where = "viewer opened on this machine"
    else:
        raise ValueError(f"unknown mode {mode!r}: use connect, save or spawn")

    log_styles()
    return where


def log_styles() -> None:
    """Names and colours, logged once as static data - not part of the timeline."""
    for n in JOINT_NAMES:
        rr.log(f"joints/{n}", rr.SeriesLines(colors=[_joint_colour(n), GREY],
                                             names=["actual", "reference"]), static=True)

    per_joint = dict(colors=[_joint_colour(n) for n in JOINT_NAMES], names=list(JOINT_NAMES))
    for path in ("policy/residual", "torque", "velocity"):
        rr.log(path, rr.SeriesLines(**per_joint), static=True)

    rr.log("estimator/vel_hdg", rr.SeriesLines(names=["lateral", "forward", "vertical"]),
           static=True)
    rr.log("estimator/pelvis_z", rr.SeriesLines(colors=[[230, 180, 60]], names=["pelvis_z"]),
           static=True)
    rr.log("imu/gyro", rr.SeriesLines(colors=AXES, names=["x", "y", "z"]), static=True)
    rr.log("imu/accel", rr.SeriesLines(colors=AXES, names=["x", "y", "z"]), static=True)
    rr.log("contacts", rr.SeriesLines(names=list(CONTACT_NAMES)), static=True)
    rr.log("feet", rr.SeriesLines(colors=[RIGHT, LEFT], names=["right_z", "left_z"]),
           static=True)
    rr.log("world/pelvis/body", rr.Boxes3D(half_sizes=[[0.08, 0.12, 0.05]],
                                           colors=[[120, 120, 130]]), static=True)
    if hasattr(rr, "TransformAxes3D"):
        rr.log("world/pelvis", rr.TransformAxes3D(0.15), static=True)


def set_time(seq: int) -> None:
    rr.set_time(TIMELINE, duration=int(seq) / 1000.0)


def _f(values) -> list:
    return [float(v) for v in values]


RAD_TO_DEG = 180.0 / math.pi


def log_state(s, degrees: bool = False) -> None:
    """Log one state onto the robot timeline. degrees=True logs joint angles
    and velocities in degrees (and deg/s) instead of radians."""
    set_time(s.seq)
    k = RAD_TO_DEG if degrees else 1.0

    pos = [v * k for v in _f(s.joint_pos)]
    ref = [v * k for v in _f(s.ref_angle)]
    for i, n in enumerate(JOINT_NAMES):
        rr.log(f"joints/{n}", rr.Scalars([pos[i], ref[i]]))

    rr.log("velocity", rr.Scalars([v * k for v in _f(s.joint_vel)]))
    rr.log("torque", rr.Scalars(_f(s.act_torque)))

    rr.log("estimator/pelvis_z", rr.Scalars([float(s.pelvis_z)]))
    rr.log("estimator/vel_hdg", rr.Scalars(_f(s.vel_hdg)))
    rr.log("estimator/fused_valid", rr.Scalars([float(s.fused_valid)]))

    rr.log("imu/gyro", rr.Scalars(_f(s.imu_gyro)))
    rr.log("imu/accel", rr.Scalars(_f(s.imu_accel)))

    rr.log("contacts", rr.Scalars(_f(s.contact)))
    rr.log("feet", rr.Scalars(_f(s.foot_z)))
    rr.log("springs", rr.Scalars(_f(s.spring_angle)))
    rr.log("gait/phase", rr.Scalars([float(s.phase)]))

    rr.log("status/safety_state", rr.Scalars([float(s.safety_state)]))
    rr.log("status/health_bits", rr.Scalars([float(s.health)]))
    rr.log("status/loop_us_max", rr.Scalars([float(s.loop_us_max)]))
    rr.log("status/overruns", rr.Scalars([float(s.overruns)]))

    # ---- 3D: the pelvis pose, and foot HEIGHTS hanging off it ----------------
    # Our quaternion is w,x,y,z; rerun wants x,y,z,w. The packet carries no foot
    # x/y, so feet sit under the hips: this shows height, not stride length.
    w, x, y, z = _f(s.quat)
    px, py = float(s.fused_pos[0]), float(s.fused_pos[1])
    rr.log("world/pelvis", rr.Transform3D(translation=[px, py, float(s.pelvis_z)],
                                          quaternion=rr.Quaternion(xyzw=[x, y, z, w])))

    feet, colours = [], []
    for fz, dy, col in ((float(s.foot_z[0]), -HIP_OFFSET_Y, RIGHT),
                        (float(s.foot_z[1]), HIP_OFFSET_Y, LEFT)):
        if math.isfinite(fz):
            feet.append([px, py + dy, fz])
            colours.append(col)
    rr.log("world/feet", rr.Points3D(feet, colors=colours, radii=0.02))


def log_residual(seq: Optional[int], residual_rad: Sequence[float]) -> None:
    """Log a command the policy sent, at the time of the newest state seen."""
    if seq is not None:
        set_time(seq)
    rr.log("policy/residual", rr.Scalars(_f(residual_rad)))
