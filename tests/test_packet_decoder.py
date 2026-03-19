"""
test_packet_decoder.py
-----------------------
Unit tests for the devibot UART packet decoder.

Run:
    cd devibot-ros2-tools
    python3 -m pytest tests/test_packet_decoder.py -v

Peribott Dynamic LLP — Hyderabad, India
"""

import struct
import sys
import os
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from packet_decoder.devibot_packet_decoder import (
    DevibotPacketDecoder,
    PacketType,
    MotorTelemetry,
    BmsState,
    HealthData,
    EncoderData,
    UltrasonicData,
    PowerState,
    NavSafety,
    Heartbeat,
    DebugMessage,
    crc16_ccitt,
    SOF,
    EOF,
)


# ── Test helpers ──────────────────────────────────────────────────────────────

def make_frame(pkt_type: int, payload: bytes) -> bytes:
    """Build a valid framed packet."""
    header = bytes([pkt_type, len(payload)]) + payload
    crc = crc16_ccitt(header)
    return bytes([SOF]) + header + struct.pack("<H", crc) + bytes([EOF])


def decode_single(frame: bytes):
    """Decode a single frame and return the first packet."""
    decoder = DevibotPacketDecoder()
    decoder.feed(frame)
    packets = list(decoder.get_packets())
    assert len(packets) == 1, f"Expected 1 packet, got {len(packets)}"
    return packets[0]


# ── CRC16 tests ───────────────────────────────────────────────────────────────

class TestCrc16:
    def test_known_value(self):
        """CRC16/CCITT-FALSE of b'123456789' = 0x29B1"""
        assert crc16_ccitt(b"123456789") == 0x29B1

    def test_empty(self):
        """Empty input returns initial value 0xFFFF."""
        assert crc16_ccitt(b"") == 0xFFFF

    def test_single_byte(self):
        result = crc16_ccitt(b"\xAA")
        assert isinstance(result, int)
        assert 0 <= result <= 0xFFFF

    def test_consistency(self):
        """Same input always produces same output."""
        data = b"\x01\x0C\x78\x00\x78\x00\x96\x00\x94\x00\x23\x24\x1E\x00"
        assert crc16_ccitt(data) == crc16_ccitt(data)

    def test_different_inputs_differ(self):
        assert crc16_ccitt(b"\x01") != crc16_ccitt(b"\x02")


# ── Framing tests ─────────────────────────────────────────────────────────────

class TestFraming:
    def test_valid_frame_decoded(self):
        payload = struct.pack("<hhhhBBBB", 100, -100, 150, 148, 35, 36, 30, 0)
        frame = make_frame(0x01, payload)
        pkt = decode_single(frame)
        assert pkt.crc_valid is True
        assert pkt.packet_type == PacketType.MOTOR_TELEMETRY

    def test_invalid_crc_rejected(self):
        payload = struct.pack("<hhhhBBBB", 100, -100, 150, 148, 35, 36, 30, 0)
        frame = bytearray(make_frame(0x01, payload))
        frame[-3] ^= 0xFF  # Corrupt CRC
        decoder = DevibotPacketDecoder()
        decoder.feed(bytes(frame))
        packets = list(decoder.get_packets())
        assert len(packets) == 0
        assert decoder.stats.crc_errors == 1

    def test_missing_eof_skipped(self):
        payload = b"\x01" * 4
        frame = bytearray(make_frame(0x01, payload))
        frame[-1] = 0x99  # Wrong EOF
        decoder = DevibotPacketDecoder()
        decoder.feed(bytes(frame))
        packets = list(decoder.get_packets())
        assert len(packets) == 0

    def test_fragmented_delivery(self):
        """Decoder handles packet split across multiple feed() calls."""
        payload = struct.pack("<HHH", 500, 480, 0)
        frame = make_frame(0x05, payload)
        decoder = DevibotPacketDecoder()
        # Feed one byte at a time
        for byte in frame:
            decoder.feed(bytes([byte]))
        packets = list(decoder.get_packets())
        assert len(packets) == 1
        assert packets[0].packet_type == PacketType.ULTRASONIC

    def test_multiple_packets(self):
        """Multiple frames in a single feed are all decoded."""
        p1 = struct.pack("<HHH", 800, 820, 0)
        p2 = struct.pack("<IBBBBB", 5000, 4, 2, 6, 0, 1)
        stream = make_frame(0x05, p1) + make_frame(0x08, p2)
        decoder = DevibotPacketDecoder()
        decoder.feed(stream)
        packets = list(decoder.get_packets())
        assert len(packets) == 2
        assert packets[0].packet_type == PacketType.ULTRASONIC
        assert packets[1].packet_type == PacketType.HEARTBEAT

    def test_garbage_before_sof(self):
        """Garbage bytes before SOF are discarded cleanly."""
        payload = struct.pack("<HHH", 100, 200, 300)
        frame = b"\xFF\x12\x34\xDE\xAD" + make_frame(0x05, payload)
        pkt = decode_single(frame)
        assert pkt.packet_type == PacketType.ULTRASONIC
        assert pkt.crc_valid is True

    def test_unknown_packet_type(self):
        """Unknown packet types are decoded with UNKNOWN type, not dropped."""
        payload = b"\xDE\xAD\xBE\xEF"
        frame = make_frame(0xEE, payload)  # Not a known type
        pkt = decode_single(frame)
        assert pkt.packet_type == PacketType.UNKNOWN
        assert pkt.raw_type == 0xEE

    def test_buffer_does_not_grow_on_garbage(self):
        """Continuous garbage does not overflow the buffer."""
        decoder = DevibotPacketDecoder(max_buffer=256)
        for _ in range(1000):
            decoder.feed(b"\xFF" * 10)
        assert decoder.buffer_len <= 256


# ── Payload parser tests ──────────────────────────────────────────────────────

class TestMotorTelemetry:
    def _make(self, rpm_l=100, rpm_r=-100, cur_l=150, cur_r=148, t_l=35, t_r=36, torque=30, faults=0):
        payload = struct.pack("<hhhhBBBB", rpm_l, rpm_r, cur_l, cur_r, t_l, t_r, torque, faults)
        return decode_single(make_frame(0x01, payload))

    def test_basic(self):
        pkt = self._make()
        d: MotorTelemetry = pkt.data
        assert d.rpm_left == 100.0
        assert d.rpm_right == -100.0
        assert abs(d.current_left - 1.50) < 0.01
        assert d.torque_limit_pct == 30

    def test_fault_flags(self):
        pkt = self._make(faults=0x03)  # Both stall flags
        d: MotorTelemetry = pkt.data
        assert d.stall_left is True
        assert d.stall_right is True
        assert d.overtemp_left is False

    def test_no_faults(self):
        pkt = self._make(faults=0)
        d: MotorTelemetry = pkt.data
        assert d.stall_left is False
        assert d.stall_right is False


class TestBmsState:
    def _make(self, v=2450, cur=-1200, soc=8500, soh=9800,
              tmax=32, tmin=28, cells=16, status=0x02, err=0):
        payload = struct.pack("<HhHHbbBBB", v, cur, soc, soh, tmax, tmin, cells, status, err)
        return decode_single(make_frame(0x02, payload))

    def test_basic(self):
        pkt = self._make()
        d: BmsState = pkt.data
        assert abs(d.voltage - 24.50) < 0.01
        assert abs(d.current - (-12.00)) < 0.01
        assert abs(d.soc - 85.00) < 0.01
        assert d.cell_count == 16

    def test_charging_flag(self):
        pkt = self._make(status=0x01)
        d: BmsState = pkt.data
        assert d.is_charging is True
        assert d.is_discharging is False

    def test_fault_flag(self):
        pkt = self._make(status=0x04)
        d: BmsState = pkt.data
        assert d.has_fault is True

    def test_power_watts(self):
        pkt = self._make(v=2400, cur=500)  # 24.00V × 5.00A = 120W
        d: BmsState = pkt.data
        assert abs(d.power_watts - 120.0) < 1.0


class TestUltrasonicData:
    def test_all_echoes(self):
        payload = struct.pack("<HHH", 800, 750, 1200)
        pkt = decode_single(make_frame(0x05, payload))
        d: UltrasonicData = pkt.data
        assert d.front_left_mm == 800
        assert d.front_right_m == pytest.approx(0.75, abs=0.001)
        assert d.rear_mm == 1200

    def test_no_echo(self):
        payload = struct.pack("<HHH", 0, 0, 0)
        pkt = decode_single(make_frame(0x05, payload))
        d: UltrasonicData = pkt.data
        assert d.front_left_m is None
        assert d.rear_m is None
        assert d.min_front_m is None

    def test_one_echo(self):
        payload = struct.pack("<HHH", 500, 0, 0)
        pkt = decode_single(make_frame(0x05, payload))
        d: UltrasonicData = pkt.data
        assert d.min_front_m == pytest.approx(0.5, abs=0.001)


class TestHeartbeat:
    def test_basic(self):
        payload = struct.pack("<IBBBBB", 60000, 4, 2, 6, 0, 0x01)
        pkt = decode_single(make_frame(0x08, payload))
        d: Heartbeat = pkt.data
        assert d.uptime_ms == 60000
        assert d.uptime_seconds == pytest.approx(60.0, abs=0.01)
        assert d.firmware_version == "4.2.6"
        assert d.ros2_link_ok is True

    def test_ros2_link_lost(self):
        payload = struct.pack("<IBBBBB", 1000, 4, 2, 6, 0, 0x00)
        pkt = decode_single(make_frame(0x08, payload))
        assert pkt.data.ros2_link_ok is False


class TestNavSafety:
    def test_safe(self):
        payload = struct.pack("<BHH", 0x04, 1000, 2000)  # safe_to_move=True
        pkt = decode_single(make_frame(0x07, payload))
        d: NavSafety = pkt.data
        assert d.safe_to_move is True
        assert d.estop_active is False

    def test_hw_estop(self):
        payload = struct.pack("<BHH", 0x01, 0, 0)  # estop_hardware=True
        pkt = decode_single(make_frame(0x07, payload))
        d: NavSafety = pkt.data
        assert d.estop_hardware is True
        assert d.estop_active is True


class TestDebugMessage:
    def test_text(self):
        msg = b"Boot phase 3 complete"
        payload = bytes([0x00]) + msg  # level=INFO
        pkt = decode_single(make_frame(0x10, payload))
        d: DebugMessage = pkt.data
        assert d.level == 0
        assert d.level_name == "INFO"
        assert d.message == "Boot phase 3 complete"

    def test_error_level(self):
        msg = b"CAN bus timeout"
        payload = bytes([0x02]) + msg  # level=ERROR
        pkt = decode_single(make_frame(0x10, payload))
        d: DebugMessage = pkt.data
        assert d.level_name == "ERROR"


# ── DecoderStats tests ────────────────────────────────────────────────────────

class TestDecoderStats:
    def test_counts_increment(self):
        decoder = DevibotPacketDecoder()
        payload = struct.pack("<HHH", 100, 200, 300)
        decoder.feed(make_frame(0x05, payload))
        decoder.feed(make_frame(0x05, payload))
        list(decoder.get_packets())
        assert decoder.stats.packets_decoded == 2
        assert decoder.stats.crc_errors == 0

    def test_error_rate(self):
        decoder = DevibotPacketDecoder()
        # 1 good packet
        payload = struct.pack("<HHH", 100, 200, 300)
        decoder.feed(make_frame(0x05, payload))
        list(decoder.get_packets())
        # 0 errors → error_rate == 0
        assert decoder.stats.error_rate == 0.0

    def test_reset_clears_state(self):
        decoder = DevibotPacketDecoder()
        decoder.feed(b"\xAA\x01\x00")  # Partial frame
        decoder.reset()
        assert decoder.buffer_len == 0
        assert list(decoder.get_packets()) == []
