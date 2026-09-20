"""
What a recorded run says about a drive's gains.

The board reports, for every joint and every tick, what the drive was TOLD
(`act_target`: the reference plus the policy's residual, after the safety
envelope, the slew limit and the interpolator) and where the joint actually
WENT (`joint_pos`). Tuning is the gap between those two, and this turns that
gap into the few numbers that say which gain to change:

  stuck      the command sweeping while the joint does not move. Missing torque
             authority, and `vel_gain` is what cures it. Drive it to zero
             FIRST: until it is zero, every other number here is measuring a
             joint that is not really following at all.
  overshoot  how far past the command the joint travels where the command
             turns around. Once stiction is beaten this dominates, and it
             points at the integrator rather than at `vel_gain`.
  lag        the time shift that best lines the two traces up. Points at
             `pos_gain` - but only once `stuck` is zero, because a best-fit
             shift against a frozen trace is meaningless.
  rms/peak   the size of the error, for comparing one run against the next.
  torque     how much of the drive's capacity the joint is using. A joint near
             its limit while merely swinging in the air does not have a gain
             problem.

`zeus tune` prints these; the tests hold the maths to known traces.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

# A recording samples the board's ticks; it does not necessarily get all of
# them. Everything below works from the board's own tick counter rather than
# assuming a 1 ms step.
TICK_S = 1e-3

MIN_SAMPLES = 100           # below this there is nothing to say
MIN_TRAVEL_RAD = 0.02       # a joint that barely moved cannot be judged
STUCK_SWEEP = 0.2           # "sweeping": over this fraction of the 95th %ile speed
STUCK_STILL = 0.05          # "not moving": under this fraction of the mean speed
TURNAROUND = 0.1            # "at the ends": under this fraction of the mean speed
MAX_LAG_S = 0.05

# What counts as a problem, in radians and milliseconds. A leg joint tracking
# its command to better than a degree is doing well; 3 degrees of error on a
# joint carrying the robot's weight is not "close enough", so these are tighter
# than they look.
LOOSE_RAD = 0.02            # 1.1 deg rms: above this, tracking is loose
OVERSHOOT_RAD = 0.01        # 0.6 deg past the command at a turnaround
OVERSHOOT_RATIO = 1.5       # ...and that much more than the average error
STUCK_FRACTION = 0.05       # 5% of the sweeping samples frozen
LAG_MS = 15.0


def analyse(seq: Sequence[float], target, position, torque) -> List[Tuple[str, Optional[Dict]]]:
    """
    Per-joint metrics from one run.

    seq        the board's tick counter for each row
    target     act_target, rows x joints, NaN where nothing was being driven
    position   joint_pos, same shape
    torque     act_torque, same shape

    Returns [(index, metrics or None)] - None where that joint was never driven
    or never moved.
    """
    import numpy as np

    seq = np.asarray(seq, dtype=float)
    target = np.asarray(target, dtype=float)
    position = np.asarray(position, dtype=float)
    torque = np.asarray(torque, dtype=float)

    dseq = np.diff(seq)
    dt = float(np.median(dseq)) * TICK_S if len(dseq) and np.median(dseq) > 0 else TICK_S

    out: List[Tuple[str, Optional[Dict]]] = []
    for j in range(target.shape[1]):
        c, p, t = target[:, j], position[:, j], torque[:, j]
        live = np.isfinite(c) & np.isfinite(p)
        if live.sum() < MIN_SAMPLES:
            out.append((j, None))
            continue

        c, p, t = c[live], p[live], t[live]
        err = c - p
        travel = float(c.max() - c.min())
        if travel < MIN_TRAVEL_RAD:
            out.append((j, None))
            continue

        cvel = np.gradient(c) / dt
        pvel = np.gradient(p) / dt
        mean_speed = float(np.abs(cvel).mean()) or 1.0

        sweeping = np.abs(cvel) > STUCK_SWEEP * np.percentile(np.abs(cvel), 95)
        stuck = sweeping & (np.abs(pvel) < STUCK_STILL * mean_speed)

        # Lag: the shift, up to MAX_LAG_S, that lines the measured trace up best.
        c0, p0 = c - c.mean(), p - p.mean()
        denom = math.sqrt(float((c0 * c0).sum()) * float((p0 * p0).sum())) or 1.0
        max_shift = max(1, min(int(round(MAX_LAG_S / dt)), len(c0) - 2))
        best, best_shift = -2.0, 0
        for shift in range(max_shift + 1):
            r = float((c0[:len(c0) - shift] * p0[shift:]).sum()) / denom
            if r > best:
                best, best_shift = r, shift

        turning = np.abs(cvel) < TURNAROUND * mean_speed
        overshoot = float(np.abs(err[turning]).max()) if turning.any() else 0.0

        out.append((j, dict(
            rms=float(np.sqrt((err ** 2).mean())),
            peak=float(np.abs(err).max()),
            stuck=float(stuck.sum() / max(int(sweeping.sum()), 1)),
            lag_ms=best_shift * dt * 1000.0,
            overshoot=overshoot,
            peak_torque=float(np.abs(t).max()),
            travel=travel,
            samples=int(live.sum()),
        )))
    return out


def advise(m: Dict) -> str:
    """
    One sentence: which gain to change next.

    The order matters as much as the thresholds. A joint that sticks is not
    following at all, so its lag and overshoot describe nothing - fix that
    first, then the ends of the stroke, then the lag through the middle.
    """
    if m["stuck"] > STUCK_FRACTION:
        return "STUCK mid-stroke: raise vel_gain (double it)"
    if m["overshoot"] > OVERSHOOT_RATIO * m["rms"] and m["overshoot"] > OVERSHOOT_RAD:
        return "overshoots at the ends: lower vel_integrator_gain"
    if m["lag_ms"] > LAG_MS:
        return f"lags {m['lag_ms']:.0f} ms: raise pos_gain"
    if m["rms"] > LOOSE_RAD:
        return "tracking loose: raise pos_gain a little"
    return "tracking well"
