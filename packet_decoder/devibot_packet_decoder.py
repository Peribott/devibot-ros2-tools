"""
devibot_packet_decoder.py
--------------------------
Binary packet decoder for the STM32↔ROS2 UART communication protocol.

The devibot AMR uses a binary framing protocol over UART at 460800 baud.
This decoder handles frame synchronisation, CRC16 validation, and
structured parsing for all packet types.

Protocol Overview:
    [SOF:1][TYPE:1][LENGTH:1][PAYLOAD:N][CRC16:2][EOF:1]

    SOF  = 0xAA  (start of frame)
    EOF  = 0x55  (end of frame)
    CRC  = CRC16/CCITT over TYPE + LENGTH + PAYLOAD bytes

Packet Types:
    0x01  MSG_MOTOR_TELEMETRY   Motor RPM, current, temperature, fault flags
    0x02  MSG_BMS_STATE         Battery voltage, current, SoC, cell temps, status
    0x03  MSG_HEALTH_DATA       Encoder faults, motor faults, comm link state
    0x04  MSG_ENCODER_DATA      Left/right encoder counts and velocities
    0x05  MSG_ULTRASONIC        Front-left, front-right, rear distances (mm)
    0x06  MSG_POWER_STATE       Boot state, button state, shutdown request flag
    0x07  MSG_NAV_SAFETY        E-stop state, obstacle proximity, safe-to-move flag
    0x08  MSG_HEARTBEAT         Uptime counter, firmware version, watchdog state
    0x10  MSG_DEBUG             ASCII debug string (development builds only)
    0xF0  MSG_ACK               Acknowledgement for commands sent to STM32
    0xF1  MSG_NACK              Negative acknowledgement with error code

Usage:
    decoder = DevibotPacketDecoder()
    decoder.feed(raw_bytes_from_uart)
    for packet in decoder.get_packets():
        print(packet)

Peribott Dynamic LLP — Hyderabad, India
https://peribott.com
"""

from __future__ import annotations

import struct
import math
import logging
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterator, Optional

log = logging.getLogger(__name__)

# ── Protocol constants ────────────────────────────────────────────────────────

SOF: int = 0xAA
EOF: int = 0x55
MAX_PAYLOAD: int = 128
MIN_FRAME_LEN: int = 6  # SOF + TYPE + LEN + CRC16(2) + EOF


# ── Packet type enumeration ───────────────────────────────────────────────────

class PacketType(IntEnum):
    MOTOR_TELEMETRY = 0x01
    BMS_STATE       = 0x02
    HEALTH_DATA     = 0x03
    ENCODER_DATA    = 0x04
    ULTRASONIC      = 0x05
    POWER_STATE     = 0x06
    NAV_SAFETY      = 0x07
    HEARTBEAT       = 0x08
    DEBUG           = 0x10
    ACK             = 0xF0
    NACK            = 0xF1
    UNKNOWN         = 0xFF


# ── Structured packet payloads ────────────────────────────────────────────────

@dataclass
class MotorTelemetry:
    """0x01 — MSG_MOTOR_TELEMETRY"""
    rpm_left: float          # Motor left RPM (signed)
    rpm_right: float         # Motor right RPM (signed)
    current_left: float      # Motor left current (A)
    current_right: float     # Motor right current (A)
    temp_left: float         # Motor left temperature (°C)
    temp_right: float        # Motor right temperature (°C)
    torque_limit_pct: int    # Configured torque limit (0–100%)
    fault_flags: int         # Bitmask: bit0=stall_L, bit1=stall_R, bit2=overtemp_L, bit3=overtemp_R

    @property
    def stall_left(self) -> bool:
        return bool(self.fault_flags & 0x01)

    @property
    def stall_right(self) -> bool:
        return bool(self.fault_flags & 0x02)

    @property
    def overtemp_left(self) -> bool:
        return bool(self.fault_flags & 0x04)

    @property
    def overtemp_right(self) -> bool:
        return bool(self.fault_flags & 0x08)


@dataclass
class BmsState:
    """0x02 — MSG_BMS_STATE"""
    voltage: float           # Pack voltage (V)
    current: float           # Pack current (A, positive=discharge, negative=charge)
    soc: float               # State of charge (0.0–100.0 %)
    soh: float               # State of health (0.0–100.0 %)
    temp_cell_max: float     # Maximum cell temperature (°C)
    temp_cell_min: float     # Minimum cell temperature (°C)
    cell_count: int          # Number of cells in series
    status_flags: int        # Bitmask: bit0=charging, bit1=discharging, bit2=fault, bit3=balancing
    error_code: int          # BMS error code (0=no error)

    @property
    def is_charging(self) -> bool:
        return bool(self.status_flags & 0x01)

    @property
    def is_discharging(self) -> bool:
        return bool(self.status_flags & 0x02)

    @property
    def has_fault(self) -> bool:
        return bool(self.status_flags & 0x04)

    @property
    def is_balancing(self) -> bool:
        return bool(self.status_flags & 0x08)

    @property
    def power_watts(self) -> float:
        return self.voltage * self.current


@dataclass
class HealthData:
    """0x03 — MSG_HEALTH_DATA"""
    encoder_fault_left: bool    # Left encoder fault detected
    encoder_fault_right: bool   # Right encoder fault detected
    motor_fault_left: bool      # Left motor drive fault
    motor_fault_right: bool     # Right motor drive fault
    can_bus_fault: bool         # CANopen bus error
    uart_fault: bool            # UART communication error
    watchdog_triggered: bool    # IWDG watchdog was triggered (reset occurred)
    comm_link_ok: bool          # ROS2 heartbeat link healthy
    uptime_ms: int              # MCU uptime since last reset (ms)
    reset_reason: int           # Reset cause code (0=power-on, 1=watchdog, 2=software, 3=fault)


@dataclass
class EncoderData:
    """0x04 — MSG_ENCODER_DATA"""
    count_left: int             # Left encoder tick count (signed, cumulative)
    count_right: int            # Right encoder tick count (signed, cumulative)
    velocity_left: float        # Left wheel velocity (m/s)
    velocity_right: float       # Right wheel velocity (m/s)
    timestamp_ms: int           # MCU timestamp at capture (ms)


@dataclass
class UltrasonicData:
    """0x05 — MSG_ULTRASONIC"""
    front_left_mm: int          # Front-left sensor distance (mm), 0=no echo
    front_right_mm: int         # Front-right sensor distance (mm), 0=no echo
    rear_mm: int                # Rear sensor distance (mm), 0=no echo

    @property
    def front_left_m(self) -> Optional[float]:
        """Distance in metres, None if no echo."""
        return self.front_left_mm / 1000.0 if self.front_left_mm > 0 else None

    @property
    def front_right_m(self) -> Optional[float]:
        return self.front_right_mm / 1000.0 if self.front_right_mm > 0 else None

    @property
    def rear_m(self) -> Optional[float]:
        return self.rear_mm / 1000.0 if self.rear_mm > 0 else None

    @property
    def min_front_m(self) -> Optional[float]:
        """Minimum front distance — closer of the two front sensors."""
        fl = self.front_left_m
        fr = self.front_right_m
        if fl is None and fr is None:
            return None
        if fl is None:
            return fr
        if fr is None:
            return fl
        return min(fl, fr)


@dataclass
class PowerState:
    """0x06 — MSG_POWER_STATE"""
    boot_phase: int             # Current boot phase (0–6, see AMR Boot System docs)
    button_held_ms: int         # Duration power button has been held (ms)
    shutdown_requested: bool    # Shutdown sequence initiated
    reboot_requested: bool      # Reboot sequence initiated
    power_source: int           # 0=battery, 1=external charger, 2=USB-C debug
    low_battery_warning: bool   # SoC below configured threshold


@dataclass
class NavSafety:
    """0x07 — MSG_NAV_SAFETY"""
    estop_hardware: bool        # Physical E-stop button engaged
    estop_software: bool        # Software E-stop commanded via ROS2
    safe_to_move: bool          # Combined safety gate — True only if both E-stops clear
    obstacle_front_mm: int      # Closest front obstacle (ultrasonic, mm)
    obstacle_rear_mm: int       # Closest rear obstacle (mm)
    nav_inhibit: bool           # Navigation inhibited (not safe to move)

    @property
    def estop_active(self) -> bool:
        return self.estop_hardware or self.estop_software


@dataclass
class Heartbeat:
    """0x08 — MSG_HEARTBEAT"""
    uptime_ms: int              # MCU uptime (ms)
    firmware_major: int         # Firmware major version
    firmware_minor: int         # Firmware minor version
    firmware_patch: int         # Firmware patch version
    watchdog_state: int         # IWDG state: 0=ok, 1=approaching timeout, 2=reset
    ros2_link_ok: bool          # ROS2 heartbeat received within timeout

    @property
    def firmware_version(self) -> str:
        return f"{self.firmware_major}.{self.firmware_minor}.{self.firmware_patch}"

    @property
    def uptime_seconds(self) -> float:
        return self.uptime_ms / 1000.0


@dataclass
class DebugMessage:
    """0x10 — MSG_DEBUG (development builds only)"""
    level: int                  # 0=INFO, 1=WARN, 2=ERROR
    message: str

    @property
    def level_name(self) -> str:
        return {0: "INFO", 1: "WARN", 2: "ERROR"}.get(self.level, "UNKNOWN")


@dataclass
class DecodedPacket:
    """A fully decoded and validated packet from the STM32."""
    packet_type: PacketType
    raw_type: int
    payload_length: int
    crc_valid: bool
    data: object = None         # One of the dataclass types above, or None

    def __str__(self) -> str:
        return (
            f"DecodedPacket("
            f"type={self.packet_type.name}, "
            f"len={self.payload_length}, "
            f"crc={'OK' if self.crc_valid else 'FAIL'}, "
            f"data={self.data})"
        )


# ── CRC16 implementation ──────────────────────────────────────────────────────

def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """
    CRC16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no input/output reflection).
    Matches the STM32 HAL CRC implementation configured for devibot.
    """
    crc = initial
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ── Payload parsers ───────────────────────────────────────────────────────────

def _sanitise_float(val: float) -> float:
    """Replace NaN/Inf with 0.0 — prevents JSON serialisation failures."""
    if math.isnan(val) or math.isinf(val):
        return 0.0
    return round(val, 4)


def _parse_motor_telemetry(payload: bytes) -> MotorTelemetry:
    # Format: 2×int16 RPM, 2×int16 current (×100 mA), 2×uint8 temp, uint8 torque, uint8 faults
    # Total: 4+4+2+1+1 = 12 bytes
    if len(payload) < 12:
        raise ValueError(f"Motor telemetry payload too short: {len(payload)} < 12")
    rpm_l, rpm_r, cur_l, cur_r, temp_l, temp_r, torque, faults = struct.unpack_from("<hhhhBBBB", payload)
    return MotorTelemetry(
        rpm_left=_sanitise_float(float(rpm_l)),
        rpm_right=_sanitise_float(float(rpm_r)),
        current_left=_sanitise_float(cur_l / 100.0),
        current_right=_sanitise_float(cur_r / 100.0),
        temp_left=_sanitise_float(float(temp_l)),
        temp_right=_sanitise_float(float(temp_r)),
        torque_limit_pct=torque,
        fault_flags=faults,
    )


def _parse_bms_state(payload: bytes) -> BmsState:
    # Format: uint16 voltage(×100 mV), int16 current(×100 mA), uint16 soc(×100 %),
    #         uint16 soh(×100 %), int8 temp_max, int8 temp_min, uint8 cells,
    #         uint8 status, uint8 error
    # Total: 2+2+2+2+1+1+1+1+1 = 13 bytes
    if len(payload) < 13:
        raise ValueError(f"BMS payload too short: {len(payload)} < 13")
    v, cur, soc, soh, tmax, tmin, cells, status, err = struct.unpack_from("<HhHHbbBBB", payload)
    return BmsState(
        voltage=_sanitise_float(v / 100.0),
        current=_sanitise_float(cur / 100.0),
        soc=_sanitise_float(soc / 100.0),
        soh=_sanitise_float(soh / 100.0),
        temp_cell_max=_sanitise_float(float(tmax)),
        temp_cell_min=_sanitise_float(float(tmin)),
        cell_count=cells,
        status_flags=status,
        error_code=err,
    )


def _parse_health_data(payload: bytes) -> HealthData:
    # Format: uint16 fault_flags, uint32 uptime_ms, uint8 reset_reason
    # fault_flags bits: 0=enc_L, 1=enc_R, 2=mot_L, 3=mot_R, 4=can, 5=uart, 6=wdt, 7=comm_ok
    if len(payload) < 7:
        raise ValueError(f"Health payload too short: {len(payload)} < 7")
    flags, uptime, reset_reason = struct.unpack_from("<HIB", payload)
    return HealthData(
        encoder_fault_left=bool(flags & 0x0001),
        encoder_fault_right=bool(flags & 0x0002),
        motor_fault_left=bool(flags & 0x0004),
        motor_fault_right=bool(flags & 0x0008),
        can_bus_fault=bool(flags & 0x0010),
        uart_fault=bool(flags & 0x0020),
        watchdog_triggered=bool(flags & 0x0040),
        comm_link_ok=bool(flags & 0x0080),
        uptime_ms=uptime,
        reset_reason=reset_reason,
    )


def _parse_encoder_data(payload: bytes) -> EncoderData:
    # Format: 2×int32 counts, 2×int16 velocity(×1000 m/s), uint32 timestamp_ms
    if len(payload) < 16:
        raise ValueError(f"Encoder payload too short: {len(payload)} < 16")
    cnt_l, cnt_r, vel_l, vel_r, ts = struct.unpack_from("<iihhI", payload)
    return EncoderData(
        count_left=cnt_l,
        count_right=cnt_r,
        velocity_left=_sanitise_float(vel_l / 1000.0),
        velocity_right=_sanitise_float(vel_r / 1000.0),
        timestamp_ms=ts,
    )


def _parse_ultrasonic(payload: bytes) -> UltrasonicData:
    # Format: 3×uint16 distances in mm (0=no echo)
    if len(payload) < 6:
        raise ValueError(f"Ultrasonic payload too short: {len(payload)} < 6")
    fl, fr, rear = struct.unpack_from("<HHH", payload)
    return UltrasonicData(
        front_left_mm=fl,
        front_right_mm=fr,
        rear_mm=rear,
    )


def _parse_power_state(payload: bytes) -> PowerState:
    # Format: uint8 boot_phase, uint16 button_held_ms, uint8 flags, uint8 power_source
    if len(payload) < 5:
        raise ValueError(f"Power state payload too short: {len(payload)} < 5")
    phase, held, flags, source = struct.unpack_from("<BHBB", payload)
    return PowerState(
        boot_phase=phase,
        button_held_ms=held,
        shutdown_requested=bool(flags & 0x01),
        reboot_requested=bool(flags & 0x02),
        power_source=source,
        low_battery_warning=bool(flags & 0x04),
    )


def _parse_nav_safety(payload: bytes) -> NavSafety:
    # Format: uint8 flags, uint16 obs_front_mm, uint16 obs_rear_mm
    if len(payload) < 5:
        raise ValueError(f"Nav safety payload too short: {len(payload)} < 5")
    flags, obs_f, obs_r = struct.unpack_from("<BHH", payload)
    return NavSafety(
        estop_hardware=bool(flags & 0x01),
        estop_software=bool(flags & 0x02),
        safe_to_move=bool(flags & 0x04),
        obstacle_front_mm=obs_f,
        obstacle_rear_mm=obs_r,
        nav_inhibit=bool(flags & 0x08),
    )


def _parse_heartbeat(payload: bytes) -> Heartbeat:
    # Format: uint32 uptime_ms, uint8 maj, uint8 min, uint8 patch, uint8 wdt, uint8 flags
    if len(payload) < 8:
        raise ValueError(f"Heartbeat payload too short: {len(payload)} < 8")
    uptime, maj, minor, patch, wdt, flags = struct.unpack_from("<IBBBBB", payload)
    return Heartbeat(
        uptime_ms=uptime,
        firmware_major=maj,
        firmware_minor=minor,
        firmware_patch=patch,
        watchdog_state=wdt,
        ros2_link_ok=bool(flags & 0x01),
    )


def _parse_debug(payload: bytes) -> DebugMessage:
    # Format: uint8 level, N bytes ASCII string
    if len(payload) < 2:
        return DebugMessage(level=0, message="")
    level = payload[0]
    msg = payload[1:].decode("ascii", errors="replace").rstrip("\x00")
    return DebugMessage(level=level, message=msg)


# ── Parser dispatch table ─────────────────────────────────────────────────────

_PARSERS = {
    PacketType.MOTOR_TELEMETRY: _parse_motor_telemetry,
    PacketType.BMS_STATE:       _parse_bms_state,
    PacketType.HEALTH_DATA:     _parse_health_data,
    PacketType.ENCODER_DATA:    _parse_encoder_data,
    PacketType.ULTRASONIC:      _parse_ultrasonic,
    PacketType.POWER_STATE:     _parse_power_state,
    PacketType.NAV_SAFETY:      _parse_nav_safety,
    PacketType.HEARTBEAT:       _parse_heartbeat,
    PacketType.DEBUG:           _parse_debug,
}


# ── Main decoder class ────────────────────────────────────────────────────────

class DevibotPacketDecoder:
    """
    Stream-safe binary packet decoder for the devibot STM32↔ROS2 UART protocol.

    Thread safety: not thread-safe. Use one decoder instance per thread/coroutine.

    Example:
        decoder = DevibotPacketDecoder()
        # In your UART read loop:
        decoder.feed(uart_port.read(256))
        for packet in decoder.get_packets():
            if packet.packet_type == PacketType.BMS_STATE:
                handle_bms(packet.data)
    """

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf: bytearray = bytearray()
        self._max_buffer = max_buffer
        self._pending: list[DecodedPacket] = []
        self.stats = DecoderStats()

    def feed(self, data: bytes) -> None:
        """
        Feed raw bytes from the UART into the decoder.
        Can be called with any chunk size — the decoder handles fragmentation.
        """
        self._buf.extend(data)
        if len(self._buf) > self._max_buffer:
            # Prevent unbounded growth if sync is lost
            log.warning(
                "Decoder buffer overflow (%d bytes) — discarding oldest half",
                len(self._buf),
            )
            self._buf = self._buf[len(self._buf) // 2:]
            self.stats.buffer_overflows += 1
        self._process()

    def get_packets(self) -> Iterator[DecodedPacket]:
        """Yield all fully decoded packets received since last call."""
        while self._pending:
            yield self._pending.pop(0)

    def _process(self) -> None:
        """Find and decode all complete frames in the buffer."""
        while True:
            # Find SOF
            sof_pos = self._buf.find(SOF)
            if sof_pos < 0:
                self._buf.clear()
                return
            if sof_pos > 0:
                # Discard bytes before SOF
                log.debug("Discarding %d bytes before SOF", sof_pos)
                self.stats.bytes_discarded += sof_pos
                del self._buf[:sof_pos]

            # Need at least MIN_FRAME_LEN bytes to determine frame length
            if len(self._buf) < MIN_FRAME_LEN:
                return

            pkt_type_byte = self._buf[1]
            payload_len = self._buf[2]

            if payload_len > MAX_PAYLOAD:
                # Corrupt length field — advance past this SOF
                log.debug("Invalid payload length %d — skipping SOF", payload_len)
                self.stats.framing_errors += 1
                del self._buf[0]
                continue

            total_frame_len = 1 + 1 + 1 + payload_len + 2 + 1  # SOF+TYPE+LEN+PAYLOAD+CRC16+EOF
            if len(self._buf) < total_frame_len:
                return  # Wait for more data

            # Check EOF marker
            if self._buf[total_frame_len - 1] != EOF:
                log.debug("Missing EOF at position %d — skipping SOF", total_frame_len - 1)
                self.stats.framing_errors += 1
                del self._buf[0]
                continue

            # Extract frame
            frame = bytes(self._buf[:total_frame_len])
            payload = frame[3:3 + payload_len]
            crc_received = struct.unpack_from("<H", frame, 3 + payload_len)[0]
            crc_data = frame[1:3 + payload_len]  # TYPE + LENGTH + PAYLOAD
            crc_computed = crc16_ccitt(crc_data)
            crc_valid = crc_received == crc_computed

            if not crc_valid:
                log.warning(
                    "CRC mismatch for type=0x%02X: got=0x%04X expected=0x%04X",
                    pkt_type_byte, crc_received, crc_computed,
                )
                self.stats.crc_errors += 1
                del self._buf[0]
                continue

            # Decode payload
            try:
                pkt_type = PacketType(pkt_type_byte)
            except ValueError:
                pkt_type = PacketType.UNKNOWN

            parsed_data = None
            if pkt_type in _PARSERS:
                try:
                    parsed_data = _PARSERS[pkt_type](payload)
                except (struct.error, ValueError) as exc:
                    log.error("Failed to parse %s payload: %s", pkt_type.name, exc)
                    self.stats.parse_errors += 1

            packet = DecodedPacket(
                packet_type=pkt_type,
                raw_type=pkt_type_byte,
                payload_length=payload_len,
                crc_valid=crc_valid,
                data=parsed_data,
            )
            self._pending.append(packet)
            self.stats.packets_decoded += 1
            del self._buf[:total_frame_len]

    def reset(self) -> None:
        """Clear buffer and pending packets — use after UART reconnect."""
        self._buf.clear()
        self._pending.clear()
        log.info("Decoder reset")

    @property
    def buffer_len(self) -> int:
        return len(self._buf)


@dataclass
class DecoderStats:
    """Running statistics for the decoder — useful for diagnosing UART health."""
    packets_decoded: int = 0
    crc_errors: int = 0
    framing_errors: int = 0
    parse_errors: int = 0
    bytes_discarded: int = 0
    buffer_overflows: int = 0

    @property
    def error_rate(self) -> float:
        total = self.packets_decoded + self.crc_errors + self.framing_errors
        if total == 0:
            return 0.0
        return (self.crc_errors + self.framing_errors) / total

    def __str__(self) -> str:
        return (
            f"decoded={self.packets_decoded} "
            f"crc_err={self.crc_errors} "
            f"frame_err={self.framing_errors} "
            f"parse_err={self.parse_errors} "
            f"error_rate={self.error_rate:.1%}"
        )
