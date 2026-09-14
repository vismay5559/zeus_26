"""The copied protocol still does what the STM32 expects."""

import math
import struct

from zeus_link import nexus_proto as P


def test_sizes_and_version_match_firmware_v6():
    # link_proto.h: nexus_state_t 444, nexus_cmd_t 52, NEXUS_PROTO_VERSION 6.
    assert P.STATE_SIZE == 444
    assert P.COMMAND_SIZE == 52
    assert P.PROTO_VERSION == 6


def test_crc_is_ccitt_false():
    # CRC-16/CCITT-FALSE check value, and the fast path matches the C transcription.
    assert P.crc16(b"123456789") == 0x29B1
    data = bytes(range(256)) * 2
    assert P.crc16(data) == P._crc16_slow(data)


def test_round_trip(packet, state_values):
    for name, want in state_values.items():
        got = getattr(packet, name)
        if isinstance(want, list):
            assert len(got) == len(want), name
            for g, w in zip(got, want):
                if isinstance(w, float) and math.isnan(w):
                    assert math.isnan(g), name
                else:
                    assert g == (struct.unpack("<f", struct.pack("<f", w))[0]
                                 if isinstance(w, float) else w), name


def test_corrupt_packet_is_rejected(packet_bytes):
    bad = bytearray(packet_bytes)
    bad[100] ^= 0xFF
    assert P.NexusState.parse(bytes(bad)) is None


def test_wrong_version_is_rejected(packet_bytes):
    bad = bytearray(packet_bytes)
    bad[3] = 5
    body = bytes(bad[:-2])
    bad[-2:] = struct.pack("<H", P.crc16(body))
    assert P.NexusState.parse(bytes(bad)) is None


def test_finds_packets_across_junk_and_split_reads(packet_bytes):
    stream = bytearray(b"\x00\xA5junk" + packet_bytes[:200])
    pkt, rest = P.NexusState.find_and_parse(stream)
    assert pkt is None                       # half a packet is not a packet
    rest += packet_bytes[200:] + packet_bytes
    first, rest = P.NexusState.find_and_parse(rest)
    second, rest = P.NexusState.find_and_parse(rest)
    assert first is not None and second is not None
    assert first.seq == second.seq == 1234


def test_command_packs_to_the_firmware_layout():
    residual = [0.001 * i for i in range(10)]
    raw = P.NexusCommand(seq=77, residual=residual, flags=P.CMD_ENABLE).pack()
    assert len(raw) == 52
    sync, msg_id, version, seq, *rest = struct.unpack(P.COMMAND_FORMAT, raw)
    values, flags, crc = rest[:10], rest[10], rest[11]
    assert (sync, msg_id, version, seq, flags) == (0xA5A5, P.MSG_COMMAND, 6, 77, 1)
    assert crc == P.crc16(raw[:-2])
    assert all(abs(a - b) < 1e-7 for a, b in zip(values, residual))


def test_joint_map_is_the_confirmed_wiring():
    assert P.JOINT_NAMES == (
        "left_hip_pitch", "left_hip_roll", "left_knee_pitch", "left_ankle_pitch",
        "waist_roll",
        "right_hip_pitch", "right_hip_roll", "right_knee_pitch", "right_ankle_pitch",
        "waist_pitch",
    )
