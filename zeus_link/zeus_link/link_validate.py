"""
Prove the link really runs at 1 kHz, and say which side is at fault when it does not.

    ros2 run zeus_link link_validate                 # 30 s, the default
    ros2 run zeus_link link_validate --seconds 300   # a long soak

`link_check` prints an arrival rate, which is one number covering two
independent questions. A reading of 970 Hz can mean the board is ticking slowly
and every tick arrived, or the board is at a clean 1000 Hz and the Pi dropped
3% - opposite faults, opposite fixes, same number on the screen. Reading it
correctly means cross-checking the loss column by eye, every time.

This separates them, because each is measured against a different clock:

  tick rate     the BOARD's tick counter over wall time. seq increments once
                per 1 kHz tick and is filled in on the board, so this is the
                board's own loop rate and owes nothing to USB. Below 1000 means
                the firmware loop is missing its own ticks - which on this
                board is usually printf, since the console UART is polled and
                blocks the loop for 87 us a character.

  delivery      packets received against ticks the board says it produced. This
                is the only figure that measures the cable, the host and the
                driver. 100% with a low tick rate means the link is perfect and
                the firmware is slow.

  board clock   the board's own microsecond timer against the Pi's clock. Near
                1.0 confirms the two agree about how long a second is; a drift
                here makes every other rate on both sides suspect.

  jitter        the gaps between arrivals. The mean can be a flawless 1.000 ms
                while the stream actually arrives in 16 ms bursts, and a policy
                does not consume a mean - it wakes up and reads whatever is
                there. So the tail is what matters: the policy runs at 250 Hz,
                a 4 ms period, and any gap longer than that is a cycle spent on
                a stale packet.

Nothing is transmitted, so this is safe to run with the drives powered.
Only one program can hold the port - stop link_node first.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Sequence

from .nexus_link import NexusLink
from .ports import PortError, find_port

# What the link is expected to deliver.
NOMINAL_HZ = 1000.0
TICK_TOLERANCE = 0.02       # 2% - below this the firmware loop is losing ticks
POLICY_HZ = 250.0           # the RL policy's rate; a gap longer than its period
POLICY_PERIOD_MS = 1000.0 / POLICY_HZ   # ...costs it a cycle on stale state
CLOCK_TOLERANCE = 0.01      # 1% disagreement between the board's clock and ours

U32 = 0xFFFFFFFF


def _span(values: Sequence[int]) -> int:
    """First-to-last distance of a uint32 counter, wrap included.

    seq wraps every ~50 days and timestamp_us every ~71 minutes, so a soak test
    left running overnight must not report the wrap as a catastrophe.
    """
    return (values[-1] - values[0]) & U32 if len(values) >= 2 else 0


def summarise(arrival_s: Sequence[float], seq: Sequence[int],
              board_us: Sequence[int]) -> Dict:
    """
    Turn one recorded run into the numbers above.

    arrival_s   host monotonic time each packet was parsed
    seq         the board's tick counter from each packet
    board_us    the board's own microsecond timestamp from each packet
    """
    n = len(seq)
    if n < 2:
        return {"packets": n, "error": "not enough packets to measure anything"}

    wall_s = arrival_s[-1] - arrival_s[0]
    if wall_s <= 0:
        return {"packets": n, "error": "no time elapsed between first and last packet"}

    ticks = _span(seq)
    expected = ticks + 1                 # inclusive of both endpoints
    gaps_ms = sorted((arrival_s[i + 1] - arrival_s[i]) * 1000.0 for i in range(n - 1))

    def pct(p: float) -> float:
        return gaps_ms[min(len(gaps_ms) - 1, int(p * len(gaps_ms)))]

    board_s = _span(board_us) / 1e6

    return {
        "packets": n,
        "duration_s": wall_s,
        "tick_rate_hz": ticks / wall_s,
        "delivered_pct": 100.0 * n / expected if expected else 0.0,
        "lost": max(0, expected - n),
        "clock_ratio": (board_s / wall_s) if wall_s else 0.0,
        "gap_p50_ms": pct(0.50),
        "gap_p99_ms": pct(0.99),
        "gap_max_ms": gaps_ms[-1],
        "gaps_over_policy": sum(1 for g in gaps_ms if g > POLICY_PERIOD_MS),
    }


def verdict(m: Dict) -> List[str]:
    """One line per thing that is wrong, and why it is that side's fault."""
    if "error" in m:
        return [f"FAIL: {m['error']}"]

    out: List[str] = []

    if m["tick_rate_hz"] < NOMINAL_HZ * (1.0 - TICK_TOLERANCE):
        short = NOMINAL_HZ - m["tick_rate_hz"]
        out.append(
            f"FAIL board: the firmware loop runs at {m['tick_rate_hz']:.1f} Hz, "
            f"missing {short:.0f} ticks a second. This is on the STM32, not the "
            f"link - printf from the 1 kHz loop is the usual cause.")

    if m["lost"]:
        out.append(
            f"FAIL link: {m['lost']} of the board's ticks never arrived "
            f"({m['delivered_pct']:.3f}% delivered). Cable, host or driver - "
            f"check nothing else holds the port.")

    if abs(m["clock_ratio"] - 1.0) > CLOCK_TOLERANCE:
        out.append(
            f"WARN clocks: the board's timer runs at {m['clock_ratio']:.4f}x the "
            f"Pi's. Every rate measured on either side is off by that much.")

    if m["gaps_over_policy"]:
        out.append(
            f"WARN jitter: {m['gaps_over_policy']} gaps longer than the policy's "
            f"{POLICY_PERIOD_MS:.0f} ms period (worst {m['gap_max_ms']:.1f} ms). "
            f"Each one is a policy cycle acting on a stale packet.")

    if not out:
        out.append(f"PASS: {m['tick_rate_hz']:.1f} Hz at the board, "
                   f"{m['delivered_pct']:.3f}% delivered, worst gap "
                   f"{m['gap_max_ms']:.1f} ms.")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="auto")
    ap.add_argument("--seconds", type=float, default=30.0,
                    help="how long to measure (default 30)")
    a = ap.parse_args(argv)

    try:
        port = find_port(a.port)
    except PortError as exc:
        print(exc, file=sys.stderr)
        return 1

    arrival_s: List[float] = []
    seq: List[int] = []
    board_us: List[int] = []

    def record(pkt) -> None:
        arrival_s.append(time.monotonic())
        seq.append(pkt.seq)
        board_us.append(pkt.timestamp_us)

    print(f"measuring {port} for {a.seconds:.0f} s - nothing is sent to the board")
    with NexusLink(port, on_packet=record) as link:
        try:
            time.sleep(a.seconds)
        except KeyboardInterrupt:
            print("\nstopped early")

    m = summarise(arrival_s, seq, board_us)
    if "error" in m:
        print(f"\n{m['error']} ({link.stats})", file=sys.stderr)
        return 1

    print(f"\n  packets        {m['packets']}")
    print(f"  duration       {m['duration_s']:.1f} s")
    print(f"  board tick     {m['tick_rate_hz']:8.1f} Hz   (nominal {NOMINAL_HZ:.0f})")
    print(f"  delivered      {m['delivered_pct']:8.3f} %    ({m['lost']} lost)")
    print(f"  board clock    {m['clock_ratio']:8.4f} x    (Pi = 1.0)")
    print(f"  gap p50/p99    {m['gap_p50_ms']:.2f} / {m['gap_p99_ms']:.2f} ms")
    print(f"  gap worst      {m['gap_max_ms']:.2f} ms\n")

    lines = verdict(m)
    for line in lines:
        print(line)
    return 0 if lines[0].startswith("PASS") else 1


if __name__ == "__main__":
    sys.exit(main())
