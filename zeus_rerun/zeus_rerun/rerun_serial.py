"""
Rerun straight off the serial port - no ROS at all.

For when the link node is NOT running: a first check on a new Pi, or a laptop
with the STM32 plugged straight into it, before the Pi is involved.

    python3 -m zeus_rerun.rerun_serial --spawn               # laptop, board on USB
    ros2 run zeus_rerun rerun_serial --connect 192.168.1.50  # Pi -> laptop viewer
    ros2 run zeus_rerun rerun_serial --save run.rrd

Only one program can hold the port. Stop the link node first, or use
rerun_node instead.
"""

from __future__ import annotations

import argparse
import sys
import time

from zeus_link.nexus_link import NexusLink
from zeus_link.ports import PortError, find_port

from . import viz


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="auto",
                    help="serial device, or auto (default)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--connect", metavar="HOST", help="stream to a viewer running on HOST")
    g.add_argument("--save", metavar="FILE.rrd", help="record to a file")
    g.add_argument("--spawn", action="store_true", help="open a viewer on this machine")
    ap.add_argument("--viewer-port", type=int, default=viz.VIEWER_PORT)
    ap.add_argument("--degrees", action="store_true",
                    help="joint angles in degrees instead of radians")
    ap.add_argument("--decimate", type=int, default=10,
                    help="log every Nth packet (default 10 = 100 Hz)")
    a = ap.parse_args(argv)

    try:
        port = find_port(a.port)
    except PortError as exc:
        print(exc, file=sys.stderr)
        return 1

    mode = "connect" if a.connect else "save" if a.save else "spawn"
    print(viz.open_sink(mode, host=a.connect or "", port=a.viewer_port,
                        path=a.save or "zeus.rrd"))

    decimate = max(1, a.decimate)
    n, last_seq = 0, None
    with NexusLink(port) as link:
        print(f"reading {port} - ctrl-c to stop")
        try:
            while True:
                pkt = link.latest()
                if pkt is None or pkt.seq == last_seq:
                    time.sleep(0.0005)
                    continue
                last_seq = pkt.seq
                n += 1
                if n % decimate == 0:
                    viz.log_state(pkt, degrees=a.degrees)
                if n % 5000 == 0:
                    print(f"  {n} packets, link {link.stats}")
        except KeyboardInterrupt:
            print(f"\nstopped after {n} packets")
    return 0


if __name__ == "__main__":
    sys.exit(main())
