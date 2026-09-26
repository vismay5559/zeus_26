"""
The 1 kHz validator, against runs whose verdict is known in advance.

The whole point of this tool is to tell two opposite faults apart - a firmware
loop missing its own ticks, and a link dropping packets the firmware produced.
They look identical on the arrival-rate reading that link_check prints, so a
validator that confuses them is worse than none: it sends someone to debug
cables when the fault is a printf, or the other way round. Each case below is
built by hand with the answer written down first.
"""

import pytest

from zeus_link.link_validate import POLICY_PERIOD_MS, summarise, verdict


def run(n=3000, tick_hz=1000.0, drop_every=0, jitter_at=None, clock_ratio=1.0):
    """One synthetic run: n board ticks, some possibly never delivered."""
    arrival, seq, us = [], [], []
    dt = 1.0 / tick_hz
    for i in range(n):
        if drop_every and (i % drop_every) == 0 and i:
            continue                       # the board ticked; nothing arrived
        t = i * dt
        if jitter_at is not None and i >= jitter_at:
            t += 0.050                     # a 50 ms stall, everything after shifts
        arrival.append(t)
        seq.append(i)
        us.append(int(i * dt * 1e6 * clock_ratio) & 0xFFFFFFFF)
    return arrival, seq, us


def test_a_perfect_link_passes():
    m = summarise(*run())
    assert m["tick_rate_hz"] == pytest.approx(1000.0, abs=1.0)
    assert m["delivered_pct"] == pytest.approx(100.0)
    assert m["lost"] == 0
    assert verdict(m)[0].startswith("PASS")


def test_a_slow_firmware_loop_is_blamed_on_the_board_not_the_link():
    """963 Hz with nothing lost: the board's own loop is missing ticks."""
    m = summarise(*run(tick_hz=963.0))

    assert m["tick_rate_hz"] == pytest.approx(963.0, abs=1.0)
    assert m["lost"] == 0
    assert m["delivered_pct"] == pytest.approx(100.0)

    said = " ".join(verdict(m))
    assert "FAIL board" in said
    assert "link" not in said.split("FAIL board")[1].split(".")[0]


def test_dropped_packets_are_blamed_on_the_link_not_the_board():
    """The board ticks at a clean 1 kHz; one packet in 50 never arrives."""
    m = summarise(*run(drop_every=50))

    assert m["tick_rate_hz"] == pytest.approx(1000.0, abs=1.0)
    assert m["lost"] > 0
    assert m["delivered_pct"] < 99.0
    assert "FAIL link" in " ".join(verdict(m))


def test_a_stall_longer_than_the_policy_period_is_reported():
    """A mean of 1.000 ms hides a 50 ms hole; the policy does not consume means."""
    m = summarise(*run(jitter_at=1500))

    assert m["gap_p50_ms"] == pytest.approx(1.0, abs=0.1)
    assert m["gap_max_ms"] > POLICY_PERIOD_MS
    assert m["gaps_over_policy"] == 1
    assert "WARN jitter" in " ".join(verdict(m))


def test_a_drifting_board_clock_is_called_out():
    m = summarise(*run(clock_ratio=1.05))
    assert m["clock_ratio"] == pytest.approx(1.05, abs=0.01)
    assert "WARN clocks" in " ".join(verdict(m))


def test_counter_wrap_is_not_reported_as_catastrophic_loss():
    """seq is uint32. A soak left running must not blame the wrap."""
    base = 0xFFFFFFFF - 500
    arrival = [i * 1e-3 for i in range(1000)]
    seq = [(base + i) & 0xFFFFFFFF for i in range(1000)]
    us = [(i * 1000) & 0xFFFFFFFF for i in range(1000)]

    m = summarise(arrival, seq, us)
    assert m["lost"] == 0
    assert m["tick_rate_hz"] == pytest.approx(1000.0, abs=1.0)


def test_too_few_packets_says_so_rather_than_dividing_by_zero():
    assert "error" in summarise([], [], [])
    assert verdict(summarise([], [], []))[0].startswith("FAIL")


# ---- the port already being held -----------------------------------------
#
# Two readers on one port is the normal mistake, not an exotic one: it happens
# the first time somebody runs link_check while a launch is up. The lock makes
# it fail, which is right - but it has to fail as an ANSWER. Left raw, pyserial
# surfaces a two-deep traceback ending in "Resource temporarily unavailable",
# and the one sentence that tells you what to do is buried in it.

def test_a_port_held_elsewhere_reports_plainly_and_does_not_traceback(monkeypatch):
    import errno as _errno

    import serial

    from zeus_link import nexus_link
    from zeus_link.ports import PortError

    def busy(*a, **k):
        exc = serial.SerialException(_errno.EAGAIN, "Could not exclusively lock port")
        exc.errno = _errno.EAGAIN
        raise exc

    monkeypatch.setattr(nexus_link.serial, "Serial", busy)

    with pytest.raises(PortError) as caught:
        nexus_link.NexusLink("/dev/zeus_stm32").start()

    said = str(caught.value)
    assert "already open in another program" in said
    assert "one reader at a time" in said.lower()


def test_starting_twice_does_not_reopen_the_port(monkeypatch):
    """The CLIs open before `with`, and __enter__ starts again."""
    from types import SimpleNamespace

    from zeus_link import nexus_link

    opened = []

    def once(*a, **k):
        opened.append(1)
        return SimpleNamespace(read=lambda n: b"", in_waiting=0,
                               reset_input_buffer=lambda: None, close=lambda: None,
                               set_low_latency_mode=lambda v: None)

    monkeypatch.setattr(nexus_link.serial, "Serial", once)

    link = nexus_link.NexusLink("/dev/zeus_stm32")
    try:
        link.start()
        link.start()
        assert len(opened) == 1
    finally:
        link.stop()
