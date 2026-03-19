"""
packet_decoder_demo.py
-----------------------
Live demonstration of the devibot packet decoder.

Modes:
  1. Live UART — reads from a real serial port
  2. Replay — decodes a previously captured binary file
  3. Synthetic — generates test packets for decoder validation

Usage:
    # Live UART
    python3 examples/packet_decoder_demo.py --port /dev/ttyS0 --baud 460800

    # Replay a capture file
    python3 examples/packet_decoder_demo.py --replay capture.bin

    # Synthetic test (no hardware required)
    python3 examples/packet_decoder_demo.py --synthetic

Peribott Dynamic LLP — Hyderabad, India
"""

import argparse
import struct
import sys
import time
import logging
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# Add project root to path when running as script
import os
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


# ── Packet formatter ──────────────────────────────────────────────────────────

def format_packet(pkt) -> str:
    """Return a human-readable single-line description of a decoded packet."""
    d = pkt.data
    if d is None:
        return f"[{pkt.packet_type.name}] (no data, CRC={'OK' if pkt.crc_valid else 'FAIL'})"

    if isinstance(d, MotorTelemetry):
        return (
            f"[MOTOR] L={d.rpm_left:+6.0f}rpm R={d.rpm_right:+6.0f}rpm | "
            f"cur L={d.current_left:.2f}A R={d.current_right:.2f}A | "
            f"torque={d.torque_limit_pct}% | faults=0x{d.fault_flags:02X}"
            + (" STALL-L" if d.stall_left else "")
            + (" STALL-R" if d.stall_right else "")
        )
    elif isinstance(d, BmsState):
        status = []
        if d.is_charging:    status.append("CHG")
        if d.is_discharging: status.append("DCHG")
        if d.has_fault:      status.append("FAULT")
        if d.is_balancing:   status.append("BAL")
        return (
            f"[BMS  ] {d.voltage:.2f}V {d.current:+.2f}A | "
            f"SoC={d.soc:.1f}% SoH={d.soh:.1f}% | "
            f"T={d.temp_cell_max:.0f}°C | {','.join(status) or 'OK'}"
        )
    elif isinstance(d, HealthData):
        faults = []
        if d.encoder_fault_left:  faults.append("ENC-L")
        if d.encoder_fault_right: faults.append("ENC-R")
        if d.motor_fault_left:    faults.append("MOT-L")
        if d.motor_fault_right:   faults.append("MOT-R")
        if d.can_bus_fault:       faults.append("CAN")
        if d.uart_fault:          faults.append("UART")
        if d.watchdog_triggered:  faults.append("WDT!")
        return (
            f"[HLTH ] uptime={d.uptime_ms/1000:.0f}s | "
            f"comm={'OK' if d.comm_link_ok else 'LOST'} | "
            f"faults={','.join(faults) or 'NONE'} | "
            f"reset={d.reset_reason}"
        )
    elif isinstance(d, EncoderData):
        return (
            f"[ENC  ] L={d.count_left:+9d} R={d.count_right:+9d} | "
            f"vel L={d.velocity_left:+.3f} R={d.velocity_right:+.3f} m/s | "
            f"t={d.timestamp_ms}ms"
        )
    elif isinstance(d, UltrasonicData):
        fl = f"{d.front_left_mm}mm" if d.front_left_mm else "----"
        fr = f"{d.front_right_mm}mm" if d.front_right_mm else "----"
        r  = f"{d.rear_mm}mm"       if d.rear_mm       else "----"
        return f"[US   ] FL={fl} FR={fr} R={r}"
    elif isinstance(d, PowerState):
        flags = []
        if d.shutdown_requested: flags.append("SHUTDOWN")
        if d.reboot_requested:   flags.append("REBOOT")
        if d.low_battery_warning: flags.append("LOW-BAT")
        return (
            f"[PWR  ] phase={d.boot_phase} btn={d.button_held_ms}ms | "
            f"src={d.power_source} | {','.join(flags) or 'OK'}"
        )
    elif isinstance(d, NavSafety):
        return (
            f"[NAV  ] estop_hw={'Y' if d.estop_hardware else 'N'} "
            f"estop_sw={'Y' if d.estop_software else 'N'} "
            f"safe={'YES' if d.safe_to_move else 'NO'} | "
            f"obs F={d.obstacle_front_mm}mm R={d.obstacle_rear_mm}mm"
        )
    elif isinstance(d, Heartbeat):
        return (
            f"[HB   ] v{d.firmware_version} uptime={d.uptime_seconds:.0f}s | "
            f"wdt={d.watchdog_state} ros2={'OK' if d.ros2_link_ok else 'LOST'}"
        )
    elif isinstance(d, DebugMessage):
        return f"[DBG  ] [{d.level_name}] {d.message}"
    else:
        return f"[{pkt.packet_type.name}] raw len={pkt.payload_length}"


# ── Synthetic packet generator ────────────────────────────────────────────────

def make_packet(pkt_type: int, payload: bytes) -> bytes:
    """Build a valid framed packet with correct CRC16."""
    header = bytes([pkt_type, len(payload)]) + payload
    crc = crc16_ccitt(header)
    return bytes([SOF]) + header + struct.pack("<H", crc) + bytes([EOF])


def synthetic_stream():
    """Generate a stream of realistic synthetic packets for testing."""
    import math
    t = 0.0
    while True:
        # Motor telemetry — sinusoidal RPM to simulate movement
        rpm = int(120 * math.sin(t * 0.5))
        payload = struct.pack("<hhhhBBBB",
            rpm, rpm + 5,   # RPM L/R
            150, 148,       # Current ×100mA
            35, 36,         # Temperature °C
            30,             # Torque limit %
            0,              # No faults
        )
        yield make_packet(0x01, payload)

        # BMS state
        soc = max(20, int(100 - t * 0.1))  # Slowly discharging
        payload = struct.pack("<HhHHbbBBB",
            2450,           # Voltage ×100 = 24.50V
            -1200,          # Current ×100 = -12.00A (discharging)
            soc * 100,      # SoC ×100
            9800,           # SoH ×100 = 98.00%
            32, 28,         # Cell temp max/min
            16,             # Cell count
            0x02,           # Discharging
            0,              # No error
        )
        yield make_packet(0x02, payload)

        # Encoder data
        count = int(t * 1000)
        vel = int(0.5 * 1000)  # 0.5 m/s × 1000
        payload = struct.pack("<iihhI", count, count + 2, vel, vel, int(t * 1000))
        yield make_packet(0x04, payload)

        # Ultrasonic
        payload = struct.pack("<HHH", 800, 820, 0)  # ~80cm front, no rear echo
        yield make_packet(0x05, payload)

        # Heartbeat every ~5 iterations
        if int(t) % 5 == 0:
            payload = struct.pack("<IBBBBB", int(t * 1000), 4, 2, 6, 0, 0x01)
            yield make_packet(0x08, payload)

        t += 0.1
        time.sleep(0.1)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="devibot packet decoder demo")
    parser.add_argument("--port",      default="/dev/ttyS0", help="Serial port")
    parser.add_argument("--baud",      type=int, default=460800, help="Baud rate")
    parser.add_argument("--replay",    help="Replay a binary capture file")
    parser.add_argument("--synthetic", action="store_true", help="Use synthetic data")
    parser.add_argument("--verbose",   action="store_true", help="Show raw packet details")
    args = parser.parse_args()

    decoder = DevibotPacketDecoder()
    pkt_count = 0
    start = time.monotonic()

    print("\ndevibot Packet Decoder Demo")
    print("=" * 60)

    try:
        if args.synthetic:
            print("Mode: SYNTHETIC (no hardware required)\n")
            for frame in synthetic_stream():
                decoder.feed(frame)
                for pkt in decoder.get_packets():
                    print(format_packet(pkt))
                    pkt_count += 1

        elif args.replay:
            print(f"Mode: REPLAY — {args.replay}\n")
            with open(args.replay, "rb") as f:
                while chunk := f.read(256):
                    decoder.feed(chunk)
                    for pkt in decoder.get_packets():
                        print(format_packet(pkt))
                        pkt_count += 1

        else:
            import serial  # type: ignore
            print(f"Mode: LIVE UART — {args.port} @ {args.baud} baud\n")
            with serial.Serial(args.port, args.baud, timeout=1.0) as ser:
                while True:
                    data = ser.read(256)
                    if data:
                        decoder.feed(data)
                        for pkt in decoder.get_packets():
                            print(format_packet(pkt))
                            if args.verbose:
                                print(f"  → {pkt}")
                            pkt_count += 1

    except KeyboardInterrupt:
        pass
    except ImportError:
        print("ERROR: pyserial not installed. Run: pip3 install pyserial")
        sys.exit(1)

    elapsed = time.monotonic() - start
    print(f"\n{'=' * 60}")
    print(f"Decoded {pkt_count} packets in {elapsed:.1f}s ({pkt_count/max(elapsed,1):.1f} pkt/s)")
    print(f"Decoder stats: {decoder.stats}")


if __name__ == "__main__":
    main()
