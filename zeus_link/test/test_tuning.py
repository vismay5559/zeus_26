"""
The tuning metrics, against traces whose answer is known in advance.

A metric that is quietly wrong is worse than no metric: it sends someone to
tune the wrong gain on a real leg. So each case here is a joint behaving in one
specific way - following well, lagging, stuck, overshooting - built by hand,
with the verdict written down before the maths runs.
"""

import math

import numpy as np
import pytest

from zeus_link.tuning import advise, analyse

TICKS = 3000                      # 3 s at 1 kHz
DT = 1e-3
FREQ = 1.0                        # Hz, a slow joint sweep
AMPL = 0.5                        # rad


def sweep(n=TICKS, ampl=AMPL):
    t = np.arange(n) * DT
    return t, ampl * np.sin(2 * math.pi * FREQ * t)


def run(command, measured, torque=None):
    """One joint, as analyse() takes it."""
    seq = np.arange(len(command), dtype=float)
    cmd = np.asarray(command, dtype=float).reshape(-1, 1)
    pos = np.asarray(measured, dtype=float).reshape(-1, 1)
    trq = (np.full_like(cmd, 1.0) if torque is None
           else np.asarray(torque, dtype=float).reshape(-1, 1))
    rows = analyse(seq, cmd, pos, trq)
    assert len(rows) == 1
    return rows[0][1]


def test_a_joint_that_follows_perfectly_reports_no_problem():
    _, c = sweep()
    m = run(c, c)
    assert m["rms"] == pytest.approx(0.0, abs=1e-9)
    assert m["stuck"] == 0.0
    assert m["lag_ms"] == 0.0
    assert advise(m) == "tracking well"


def test_a_lagging_joint_is_measured_in_milliseconds():
    """The classic pos_gain symptom: right shape, arriving late."""
    _, c = sweep(TICKS + 30)
    lag = 25                                     # ticks = ms at 1 kHz
    m = run(c[lag:], c[:-lag])                   # measured is the command, delayed

    assert m["lag_ms"] == pytest.approx(lag, abs=2)
    assert m["stuck"] == 0.0
    assert "pos_gain" in advise(m)


def test_a_joint_stuck_mid_stroke_is_caught_first():
    """
    Stiction: the command sweeps, the joint sits still, then jumps to catch up.

    This has to be reported as stuck rather than as lag or overshoot - it is
    the one failure whose fix (vel_gain) is different from all the others.
    """
    _, c = sweep()
    p = c.copy()
    for start in range(200, TICKS, 500):         # freeze for 150 ms at a time
        p[start:start + 150] = p[start]

    m = run(c, p)
    assert m["stuck"] > 0.05, m
    assert "vel_gain" in advise(m)


def test_a_joint_that_overshoots_the_ends_points_at_the_integrator():
    """
    Integrator wind-up: it follows the middle of the stroke and sails past
    where the command turns around. The error therefore piles up at the ends
    rather than being spread evenly, which is what separates this from a
    joint that is simply loose.
    """
    _, c = sweep()
    p = c + 0.06 * (c / AMPL) ** 3                # error concentrated at the ends
    m = run(c, p)

    assert m["stuck"] == 0.0
    assert m["overshoot"] > 1.5 * m["rms"]
    assert "integrator" in advise(m)


def test_a_joint_that_was_never_driven_is_not_judged():
    """act_target is NaN whenever the board is not driving."""
    _, c = sweep()
    m = run(np.full(TICKS, np.nan), c)
    assert m is None


def test_a_joint_that_barely_moved_is_not_judged():
    """A millimetre of travel makes every ratio here meaningless."""
    _, c = sweep(ampl=0.001)
    m = run(c, c * 0.5)
    assert m is None


def test_metrics_use_the_boards_tick_counter_not_the_row_count():
    """
    A recording is a sample of the board's ticks, not all of them.

    With every other tick kept, a 25 ms lag is 12 rows - and reporting it as
    12 ms would send someone tuning a joint that is twice as slow as they
    think.
    """
    _, c = sweep(TICKS + 60)
    lag = 24                                              # ms
    cmd, pos = c[lag:], c[:-lag]

    cmd, pos = cmd[::2], pos[::2]                         # every other tick kept
    seq = np.arange(0, 2 * len(cmd), 2, dtype=float)      # ...and labelled as such
    rows = analyse(seq, cmd.reshape(-1, 1), pos.reshape(-1, 1), np.ones((len(cmd), 1)))
    m = rows[0][1]

    assert m["lag_ms"] == pytest.approx(lag, abs=3)


def test_peak_torque_is_reported_for_headroom():
    _, c = sweep()
    torque = np.full(TICKS, 2.0)
    torque[500] = -7.5
    m = run(c, c, torque)
    assert m["peak_torque"] == pytest.approx(7.5)
