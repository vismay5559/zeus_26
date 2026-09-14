"""Port discovery, ODrive names, and the threaded reader end to end."""

import time
from types import SimpleNamespace

import pytest
import serial

from zeus_link import nexus_link, nexus_proto as P
from zeus_link.odrive_names import axis_state_name, error_names
from zeus_link.ports import PortError, STABLE_NAME, find_port
from zeus_link.synthetic import pack_state, state_values as build_state


def _port(dev, vid=None, pid=None):
    return SimpleNamespace(device=dev, vid=vid, pid=pid)


def test_explicit_port_is_used_as_given():
    assert find_port("/dev/ttyACM3", comports=lambda: [], exists=lambda p: False) == "/dev/ttyACM3"


def test_udev_name_wins():
    assert find_port("auto", comports=lambda: [], exists=lambda p: p == STABLE_NAME) == STABLE_NAME


def test_picks_the_stm32_link_not_the_stlink():
    ports = [_port("/dev/ttyACM0", 0x0483, 0x374E),     # ST-LINK virtual COM port
             _port("/dev/ttyACM1", 0x0483, 0x5740)]     # the Appli's USB CDC
    assert find_port(comports=lambda: ports, exists=lambda p: False) == "/dev/ttyACM1"


def test_none_found_lists_what_is_there():
    with pytest.raises(PortError, match="ttyACM0"):
        find_port(comports=lambda: [_port("/dev/ttyACM0", 0x0483, 0x374E)],
                  exists=lambda p: False)


def test_two_links_is_an_error():
    ports = [_port("/dev/ttyACM0", 0x0483, 0x5740), _port("/dev/ttyACM1", 0x0483, 0x5740)]
    with pytest.raises(PortError, match="2 devices"):
        find_port(comports=lambda: ports, exists=lambda p: False)


def test_odrive_names():
    assert error_names(0x08000200) == ["DC_BUS_UNDER_VOLTAGE", "BRAKE_RESISTOR_DISARMED"]
    assert error_names(0x10000000) == ["THERMISTOR_DISCONNECTED"]
    assert error_names(0x80000000) == ["0x80000000"]
    assert axis_state_name(8) == "CLOSED_LOOP_CONTROL"
    assert axis_state_name(99) == "state 99"


def test_reader_thread_receives_packets_and_sends_commands(monkeypatch):
    # A pyserial loopback stands in for the STM32: packets written into it come
    # back out of the reader thread, and commands the link writes can be read.
    loop = serial.serial_for_url("loop://", timeout=0.005)
    monkeypatch.setattr(nexus_link.serial, "Serial", lambda *a, **k: loop)

    link = nexus_link.NexusLink("loop://", history=100)
    link.start()
    try:
        for seq in range(10, 15):
            loop.write(pack_state(build_state(seq=seq)))
        deadline = time.monotonic() + 2.0
        while link.stats.packets < 5 and time.monotonic() < deadline:
            time.sleep(0.01)

        assert link.stats.packets == 5
        assert link.latest().seq == 14
        assert [p.seq for p in link.drain()] == [10, 11, 12, 13, 14]
        assert link.stats.seq_lost == 0

        # a gap in seq is counted as lost
        loop.write(pack_state(build_state(seq=20)))
        deadline = time.monotonic() + 2.0
        while link.stats.packets < 6 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert link.stats.seq_lost == 5
    finally:
        link.stop()


def test_send_command_writes_one_valid_frame(monkeypatch):
    written = bytearray()
    fake = SimpleNamespace(write=written.extend, read=lambda n: b"", in_waiting=0,
                           reset_input_buffer=lambda: None, close=lambda: None)
    monkeypatch.setattr(nexus_link.serial, "Serial", lambda *a, **k: fake)
    link = nexus_link.NexusLink("fake")
    link._ser = fake                               # no reader thread needed
    link.send_command([0.01] * 10, enable=True)
    link.send_command([0.0] * 10, enable=False)
    assert len(written) == 2 * P.COMMAND_SIZE
    import struct
    first = struct.unpack(P.COMMAND_FORMAT, bytes(written[:52]))
    second = struct.unpack(P.COMMAND_FORMAT, bytes(written[52:]))
    assert first[3] == 0 and second[3] == 1        # seq advances per command
    assert first[-2] & P.CMD_ENABLE and not (second[-2] & P.CMD_ENABLE)
