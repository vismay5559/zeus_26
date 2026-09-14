"""
Shared fixtures. Runs on any machine: nothing here needs ROS.

`packet_bytes` builds a real 444-byte state packet the way the STM32 does, so
every test below exercises the actual parser rather than a stand-in.
"""

import math
import os
import re
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
WS = os.path.dirname(PKG)
sys.path.insert(0, PKG)

from zeus_link import nexus_proto as P  # noqa: E402
from zeus_link.synthetic import pack_state, state_values as build_state  # noqa: E402,F401

MSG_DIR = os.path.join(WS, "zeus_msgs", "msg")


@pytest.fixture
def state_values():
    return build_state()


@pytest.fixture
def packet_bytes(state_values):
    return pack_state(state_values)


@pytest.fixture
def packet(packet_bytes):
    pkt = P.NexusState.parse(packet_bytes)
    assert pkt is not None
    return pkt


# --------------------------------------------------------------------------
# .msg parsing, for checking the ROS definitions without ROS
# --------------------------------------------------------------------------

_FIELD = re.compile(r"^\s*([A-Za-z0-9_/]+?)(?:\[(\d+)\])?\s+([a-z][a-z0-9_]*)\s*(?:#.*)?$")
_CONST = re.compile(r"^\s*([a-z0-9]+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(\S+)")


def read_msg(name):
    """(fields, constants): fields as [(type, count, name)], count 1 for scalars."""
    fields, consts = [], {}
    with open(os.path.join(MSG_DIR, name + ".msg"), encoding="utf-8") as f:
        for line in f:
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            m = _CONST.match(line)
            if m:
                consts[m.group(2)] = int(m.group(3), 0)
                continue
            m = _FIELD.match(line)
            assert m, f"unparsed line in {name}.msg: {line!r}"
            fields.append((m.group(1), int(m.group(2)) if m.group(2) else 1, m.group(3)))
    return fields, consts


class StrictMsg:
    """
    Mimics the type checks rclpy's generated message classes make.

    rosidl_generator_py asserts that a float32 scalar is a Python float, an
    integer scalar a Python int, and a fixed numeric array a numpy array of the
    exact dtype and size (or a sequence of the right element type). Handing it
    a numpy float32 scalar, or an int where a float is wanted, raises on the
    robot. This raises here instead.
    """

    _NP = {"float32": "float32", "uint32": "uint32", "uint16": "uint16", "uint8": "uint8"}

    def __init__(self, msg_name):
        object.__setattr__(self, "_spec", {n: (t, c) for t, c, n in read_msg(msg_name)[0]})

    def __setattr__(self, name, value):
        import numpy as np

        spec = self._spec
        assert name in spec, f"no field {name!r} in the message"
        typ, count = spec[name]
        if count == 1:
            if typ in ("float32", "float64"):
                assert type(value) is float, f"{name}: needs float, got {type(value).__name__}"
                assert math.isnan(value) or abs(value) <= 3.402823466e38 or math.isinf(value)
            elif typ.startswith("uint") or typ.startswith("int"):
                assert type(value) is int, f"{name}: needs int, got {type(value).__name__}"
                bits = int(typ.lstrip("uint"))
                assert 0 <= value < (1 << bits), f"{name}: {value} out of range for {typ}"
            elif typ == "bool":
                assert type(value) is bool
        else:
            if isinstance(value, np.ndarray):
                assert value.dtype == np.dtype(self._NP[typ]), \
                    f"{name}: dtype {value.dtype}, needs {typ}"
                assert value.size == count, f"{name}: {value.size} values, needs {count}"
            else:
                assert len(value) == count, f"{name}: {len(value)} values, needs {count}"
                want = float if typ.startswith("float") else int
                assert all(type(x) is want for x in value), f"{name}: element type"
        object.__setattr__(self, name, value)
