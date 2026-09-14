"""
Check the USB link without ROS - the first thing to run on a new Pi.

    ros2 run zeus_link link_check              # finds the board
    ros2 run zeus_link link_check /dev/ttyACM1

Prints once a second. A healthy link reads ~1000 Hz and 0.000% lost. Anything
less points at the Pi - scheduling, the tty layer, another program holding the
port - and is far easier to find here than inside a control loop.

Nothing is sent to the board, so this is safe with the drives powered. Stop the
link node first: only one program can hold the port.
"""

from __future__ import annotations

import argparse
import sys
import time

from .nexus_link import NexusLink
from .nexus_proto import JOINT_NAMES, SAFETY_NAMES
from .odrive_names import axis_state_name, error_names
from .ports import PortError, find_port

FUSION_NAMES = {0: "INVALID", 1: "CONVERGING", 2: "OK"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", nargs="?", default="auto")
    ap.add_argument("--joints", action="store_true",
                    help="also print every joint's position, state and error")
    a = ap.parse_args(argv)

    try:
        port = find_port(a.port)
    except PortError as exc:
        print(exc, file=sys.stderr)
        return 1

    with NexusLink(port) as link:
        print(f"reading {port} - ctrl-c to stop\n")
        t0, last = time.monotonic(), 0
        try:
            while True:
                time.sleep(1.0)
                now = time.monotonic()
                rate = (link.stats.packets - last) / (now - t0)
                t0, last = now, link.stats.packets

                pkt = link.latest()
                if pkt is None:
                    print("no packets - is the Appli running in NEXUS_MODE_ROBOT?")
                    continue

                faults = ",".join(pkt.faults()) or "none"
                print(f"{rate:7.1f} Hz | seq {pkt.seq:9d} | {link.stats} | "
                      f"{SAFETY_NAMES.get(pkt.safety_state, '?'):5s} | "
                      f"fusion {FUSION_NAMES.get(pkt.fused_valid, '?'):10s} | "
                      f"phase {pkt.phase:.3f} | faults {faults}")

                if a.joints:
                    for j, name in enumerate(JOINT_NAMES):
                        err = pkt.act_error[j]
                        print(f"    {name:18s} pos {pkt.joint_pos[j]:+8.4f} rad  "
                              f"ref {pkt.ref_angle[j]:+8.4f}  "
                              f"{axis_state_name(pkt.act_state[j]):20s}"
                              + (" " + "|".join(error_names(err)) if err else ""))
        except KeyboardInterrupt:
            print(f"\nfinal: {link.stats}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
