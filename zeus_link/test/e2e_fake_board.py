"""
End-to-end: the real ROS nodes against fake_board, over a pty.

Needs a built and sourced workspace, so it is run by CI's colcon job and by
hand, not by pytest:

    source install/setup.bash
    python3 zeus_link/test/e2e_fake_board.py

Checks the two failures this was written to catch, both of which unit tests
missed and a real board would have shown:

  1. /zeus/state keeps publishing after the board reports FAULT. It used to
     stop dead: a logger call raised inside the publish thread.
  2. A policy restarted after a fault re-arms the board. It used to stay
     latched, because the enable-false handshake was sent before DDS discovery
     had connected the policy to the link node.
  3. A restarted LINK NODE re-arms the board, although its command counter
     starts from 0 again. The firmware used to reject that as stale until
     rebooted; fake_board models the fixed rule (safety.c s_seq_valid).

Plus: the stream reaches 1 kHz, commands arrive intact and in sequence, the
stand-down service idles the board, and stopping the link node stands it down.
"""

import json
import os
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix

LIB = os.path.join(get_package_prefix("zeus_link"), "lib", "zeus_link")
failures = []


def check(ok, what):
    print(("  ok    " if ok else "  FAIL  ") + what, flush=True)
    if not ok:
        failures.append(what)


def node(name, *args, log):
    return subprocess.Popen([os.path.join(LIB, name), *args], stdout=log,
                            stderr=subprocess.STDOUT, env=ENV)


def stop(p):
    p.send_signal(signal.SIGINT)
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()


tmp = tempfile.mkdtemp(prefix="zeus_e2e_")
link = os.path.join(tmp, "board")
summary = os.path.join(tmp, "summary.json")
ENV = dict(os.environ, ROS_DOMAIN_ID=str(40 + os.getpid() % 50),
           ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", PYTHONUNBUFFERED="1")
logs = {n: open(os.path.join(tmp, n + ".log"), "w+")
        for n in ("board", "link", "policy1", "policy2", "policy3", "hz", "service")}

board = node("fake_board", "--link", link, "--summary", summary, "--duration", "60",
             log=logs["board"])
for _ in range(50):
    if os.path.exists(link):
        break
    time.sleep(0.1)

link_node = node("link_node", "--ros-args", "-p", f"port:={link}",
                 "-p", "stats_period_sec:=1.0", log=logs["link"])
time.sleep(3)

print("stream rate:", flush=True)
subprocess.run(["timeout", "5", "ros2", "topic", "hz", "/zeus/state"],
                    stdout=logs["hz"], stderr=subprocess.STDOUT, env=ENV)
logs["hz"].seek(0)
rates = [float(l.split(":")[1]) for l in logs["hz"] if l.startswith("average rate")]
check(bool(rates) and rates[-1] > 900, f"/zeus/state at {rates[-1] if rates else 0:.1f} Hz")

print("arm, stand down, resume:", flush=True)
policy = node("gait_passthrough_node", "--ros-args", "-p", "enable:=true", log=logs["policy1"])
time.sleep(4)
subprocess.run(["timeout", "15", "ros2", "service", "call", "/zeus/stand_down",
                "std_srvs/srv/Trigger"], stdout=logs["service"], stderr=subprocess.STDOUT, env=ENV)
time.sleep(1)
subprocess.run(["timeout", "15", "ros2", "service", "call", "/zeus/resume",
                "std_srvs/srv/Trigger"], stdout=logs["service"], stderr=subprocess.STDOUT, env=ENV)
time.sleep(2)

print("kill the policy -> FAULT, restart -> must re-arm:", flush=True)
stop(policy)
time.sleep(2)
policy = node("gait_passthrough_node", "--ros-args", "-p", "enable:=true", log=logs["policy2"])
time.sleep(6)

print("restart the link node too (its seq counts from 0) -> must re-arm:", flush=True)
stop(policy)
time.sleep(0.1)
stop(link_node)
time.sleep(1)
link_node = node("link_node", "--ros-args", "-p", f"port:={link}",
                 "-p", "stats_period_sec:=1.0", log=logs["link"])
policy = node("gait_passthrough_node", "--ros-args", "-p", "enable:=true", log=logs["policy3"])
time.sleep(8)

stop(policy)
time.sleep(0.1)
stop(link_node)
time.sleep(0.5)
board.send_signal(signal.SIGTERM)
board.wait(timeout=10)

s = json.load(open(summary))
events = [(e[1], e[2]) for e in s["events"]]
print("board transitions:", " | ".join(f"{e[0]:.2f}s {e[1]}->{e[2]}" for e in s["events"]))

check(s["crc_bad"] == 0, f"{s['commands']} commands, {s['crc_bad']} with a bad CRC")
check(s["seq_gaps"] == 1, f"command sequence gaps: {s['seq_gaps']} (want 1: the link node restart)")
check(("IDLE", "ARMED") in events, "first policy armed the board")
check(("ARMED", "IDLE") in events, "stand_down idled the board")
fault = [i for i, e in enumerate(events) if e[1] == "FAULT"]
check(bool(fault), "stopping the policy faulted the board")
check(bool(fault) and ("IDLE", "ARMED") in events[fault[0]:],
      "restarted policy cleared the latch and re-armed")
check(s["final"] == "IDLE", f"link node shutdown left the board {s['final']}")
arm_times = [e[0] for e in s["events"] if (e[1], e[2]) == ("IDLE", "ARMED")]
check(len(arm_times) >= 4, f"restarted link node re-armed the board ({len(arm_times)} arms; want 4: "
      "first policy, resume, restarted policy, restarted link node)")

link_log = open(os.path.join(tmp, "link.log")).read()
check("Traceback" not in link_log, "no exception in link_node")
lines = [l for l in link_log.splitlines() if "published" in l]
after_fault = [l for l in lines[-6:] if "published    0.0 Hz" in l]
check(not after_fault, "/zeus/state still publishing after the FAULT")
policy2 = open(os.path.join(tmp, "policy2.log")).read()
check("handshake confirmed" in policy2, "restarted policy reports a confirmed handshake")

print(f"\nlogs in {tmp}")
if failures:
    print(f"{len(failures)} FAILED")
    sys.exit(1)
print("END-TO-END PASSED")
