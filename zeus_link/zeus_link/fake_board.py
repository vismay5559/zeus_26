"""
A fake STM32 on a pseudo-terminal: run the whole Pi stack with no robot.

    ros2 run zeus_link fake_board                        # prints the port to use
    ros2 launch zeus_bringup walk.launch.py port:=/tmp/zeus_fake_board enable:=true

Linux and macOS only (it needs a pty).

It streams byte-exact 444-byte state packets at 1 kHz - a smooth fake stride
in joint_pos and ref_angle, the stride clock in phase - and parses every
command packet that comes back, so everything between the USB port and your
policy runs for real: link_node, the messages, QoS, the policy, rerun.

It models the parts of the firmware's safety.c the Pi has to get right:

  BOOT -> IDLE     commands with enable false arrive; clears the re-arm latch
  IDLE -> ARMED    enable true, with the latch clear and |residual| <= 0.1 turn
  IDLE/ARMED -> FAULT   200 ms without a command; sets the latch
  sequence         an enabled command must advance seq past the last accepted
                   one; enable false starts a new session (a restarted link
                   node counts from 0 again). 10 rejections in a row -> FAULT.

It does NOT model the robot: joint_pos does not follow your residual, and the
estimator values are fixed. It tests plumbing and protocol, not a policy's
control quality.

With --summary it writes what it saw on exit - commands, CRC errors, sequence
gaps, every safety transition - which is what the CI end-to-end test checks.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import struct
import sys
import threading
import time
import tty

from . import nexus_proto as P
from .synthetic import pack_state, state_values

DEFAULT_LINK = "/tmp/zeus_fake_board"
RESIDUAL_LIMIT_TURNS = 0.1
LINK_TIMEOUT_S = 0.2
MAX_REJECTS = 10


class FakeBoard:

    def __init__(self, link_path: str, error_at: float = -1.0) -> None:
        self.master, slave = os.openpty()
        tty.setraw(slave)
        self._slave = slave
        self.link_path = link_path
        if os.path.lexists(link_path):
            os.remove(link_path)
        os.symlink(os.ttyname(slave), link_path)

        self.error_at = error_at
        self.lock = threading.Lock()
        self.t0 = time.monotonic()
        self.safety = P.SAFETY_BOOT
        self.gains_seq = 0
        self.needs_rearm = False
        self.last_cmd = None
        self.gains = None
        self.stats = dict(commands=0, crc_bad=0, seq_gaps=0, enables=0, disables=0,
                          gains=0, gains_refused=0,
                          rejected=0, packets_sent=0)
        self.events = []
        self._last_cmd_seq = None
        self._accepted_seq = None       # None = new session, like s_seq_valid
        self._reject_run = 0
        self._stop = threading.Event()

    # ---- state machine ------------------------------------------------------

    def _set(self, new: int, why: str) -> None:
        if self.safety != new:
            t = round(time.monotonic() - self.t0, 3)
            self.events.append([t, P.SAFETY_NAMES[self.safety], P.SAFETY_NAMES[new], why])
            print(f"  {t:8.3f} s  {P.SAFETY_NAMES[self.safety]} -> {P.SAFETY_NAMES[new]}  ({why})",
                  flush=True)
            self.safety = new

    def _on_command(self, frame: bytes) -> None:
        f = struct.unpack(P.COMMAND_FORMAT, frame)
        seq, residual, flags = f[3], f[4:4 + P.NUM_JOINTS], f[4 + P.NUM_JOINTS]
        s = self.stats
        if self._last_cmd_seq is not None and seq != (self._last_cmd_seq + 1) & 0xFFFFFFFF:
            s["seq_gaps"] += 1
        self._last_cmd_seq = seq
        s["commands"] += 1
        self.last_cmd = time.monotonic()

        if not flags & P.CMD_ENABLE:
            s["disables"] += 1
            self.needs_rearm = False
            self._accepted_seq = None
            self._set(P.SAFETY_IDLE, "enable false")
            return

        s["enables"] += 1
        if self.needs_rearm or self.safety in (P.SAFETY_BOOT, P.SAFETY_FAULT):
            return
        stale = (self._accepted_seq is not None and
                 struct.unpack("<i", struct.pack("<I", (seq - self._accepted_seq) & 0xFFFFFFFF))[0] <= 0)
        bad = any(not math.isfinite(r) or abs(r) > RESIDUAL_LIMIT_TURNS for r in residual)
        if stale or bad:
            s["rejected"] += 1
            self._reject_run += 1
            if self._reject_run >= MAX_REJECTS:
                self.needs_rearm = True
                self._set(P.SAFETY_FAULT, f"{MAX_REJECTS} rejected commands")
            return
        self._reject_run = 0
        self._accepted_seq = seq
        self._set(P.SAFETY_ARMED, "enable true")

    def _on_gains(self, frame: bytes) -> None:
        """Gains land only between runs, exactly as the board treats them."""
        f = struct.unpack(P.GAINS_FORMAT, frame)
        seq = f[3]
        self.stats["gains"] += 1

        if self.safety == P.SAFETY_ARMED:
            self.stats["gains_refused"] += 1
            return

        self.gains = dict(pos=f[4:4 + P.NUM_JOINTS],
                          vel=f[4 + P.NUM_JOINTS:4 + 2 * P.NUM_JOINTS],
                          vel_int=f[4 + 2 * P.NUM_JOINTS:4 + 3 * P.NUM_JOINTS])
        self.gains_seq = seq & 0xFF

    # ---- threads --------------------------------------------------------------

    def _reader(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                buf += os.read(self.master, 4096)
            except OSError:
                return
            while True:
                i = buf.find(P.SYNC_BYTES)
                if i < 0:
                    del buf[:-1]
                    break
                if len(buf) - i < 3:
                    del buf[:i]
                    break
                # Two message types, two lengths - the real board decides the
                # same way, from the id in the third byte.
                size = {P.MSG_COMMAND: P.COMMAND_SIZE, P.MSG_GAINS: P.GAINS_SIZE}.get(buf[i + 2])
                if size is None:
                    del buf[:i + 1]
                    continue
                if len(buf) - i < size:
                    del buf[:i]
                    break
                frame = bytes(buf[i:i + size])
                if P.crc16(frame[:-2]) != struct.unpack_from("<H", frame, size - 2)[0]:
                    self.stats["crc_bad"] += 1
                    del buf[:i + 1]
                    continue
                del buf[:i + size]
                with self.lock:
                    if frame[2] == P.MSG_COMMAND:
                        self._on_command(frame)
                    else:
                        self._on_gains(frame)

    def run(self, duration: float) -> None:
        threading.Thread(target=self._reader, daemon=True).start()
        seq, nxt = 0, time.monotonic()
        while not self._stop.is_set() and (duration <= 0 or time.monotonic() - self.t0 < duration):
            now = time.monotonic()
            with self.lock:
                link_ok = self.last_cmd is not None and now - self.last_cmd < LINK_TIMEOUT_S
                if not link_ok and self.safety in (P.SAFETY_IDLE, P.SAFETY_ARMED):
                    self.needs_rearm = True
                    self._set(P.SAFETY_FAULT, "no command for 200 ms")
                safety = self.safety

            phase = (seq / (1000.0 * 1.26)) % 1.0
            w = 2.0 * math.pi * phase
            errors = [0] * P.NUM_JOINTS
            if self.error_at >= 0 and self.error_at <= now - self.t0 < self.error_at + 1.0:
                errors[P.JOINT_INDEX["right_ankle_pitch"]] = 0x08000200
            pkt = pack_state(state_values(
                seq=seq, timestamp_us=(seq * 1000) & 0xFFFFFFFF, phase=phase,
                joint_pos=[0.2 * math.sin(w + j) for j in range(P.NUM_JOINTS)],
                joint_vel=[0.2 * w * math.cos(w + j) for j in range(P.NUM_JOINTS)],
                ref_angle=[0.2 * math.sin(w + j + 0.05) for j in range(P.NUM_JOINTS)],
                health=0 if link_ok else P.HEALTH_LINK, safety_state=safety,
                act_state=[8 if safety == P.SAFETY_ARMED else 1] * P.NUM_JOINTS,
                # What the drives were told: the reference the board is playing
                # while armed, and NaN while nothing is being driven.
                act_target=([0.2 * math.sin(w + j + 0.05) for j in range(P.NUM_JOINTS)]
                            if safety == P.SAFETY_ARMED
                            else [float("nan")] * P.NUM_JOINTS),
                gains_seq=self.gains_seq,
                act_error=errors))
            try:
                os.write(self.master, pkt)
            except OSError:
                pass
            seq += 1
            nxt += 0.001
            time.sleep(max(0.0, nxt - time.monotonic()))
        self.stats["packets_sent"] = seq

    def stop(self) -> None:
        self._stop.set()

    def close(self) -> None:
        if os.path.lexists(self.link_path):
            os.remove(self.link_path)
        os.close(self.master)
        os.close(self._slave)

    def summary(self) -> dict:
        return dict(self.stats, events=self.events, final=P.SAFETY_NAMES[self.safety],
                    elapsed_s=round(time.monotonic() - self.t0, 2))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--link", default=DEFAULT_LINK, help="symlink to create for the pty")
    ap.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until ctrl-c")
    ap.add_argument("--summary", help="write a JSON summary here on exit")
    ap.add_argument("--error-at", type=float, default=-1.0,
                    help="report an ODrive error on right_ankle_pitch for 1 s from this time")
    a = ap.parse_args(argv)

    board = FakeBoard(a.link, error_at=a.error_at)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: board.stop())
    print(f"fake STM32 streaming 1 kHz on {a.link}\n"
          f"  ros2 launch zeus_bringup walk.launch.py port:={a.link}", flush=True)
    try:
        board.run(a.duration)
    finally:
        s = board.summary()
        board.close()
        if a.summary:
            with open(a.summary, "w") as f:
                json.dump(s, f, indent=1)
        print(f"sent {s['packets_sent']} packets, received {s['commands']} commands, "
              f"{s['crc_bad']} bad, final state {s['final']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
