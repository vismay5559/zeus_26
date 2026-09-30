"""
The observation table in zeus_control_interface/README.md is what a policy
author builds the network's input from. It went stale across two protocol
versions - still describing ten joints and 52 values when the packet carried
eight and 44 - and nothing noticed, because nothing read it but people.

A wrong slice here does not crash. It feeds joint velocities into the inputs
the network learned as joint positions, and the robot simply walks badly.
"""

import os
import re

from zeus_link import nexus_proto as P

README = os.path.join(os.path.dirname(__file__), "..", "..",
                      "zeus_control_interface", "README.md")


def test_this_table_matches_the_protocol():
    text = open(README, encoding="utf-8").read()
    rows = re.findall(r"^\| (\d+)(?:–(\d+))? \| `(\w+)` \| (\d+) \|", text, flags=re.M)

    table = [(name, int(lo), int(hi or lo), int(size)) for lo, hi, name, size in rows]
    assert [name for name, *_ in table] == [name for name, _ in P.POLICY_FIELDS]

    for name, lo, hi, size in table:
        off, n = P.POLICY_INDEX[name]
        assert (lo, hi, size) == (off, off + n - 1, n), name

    assert f"gives the {P.POLICY_COUNT} values" in text
    assert f"[0.0] * {P.NUM_JOINTS}" in text
