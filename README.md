# devibot ROS2 Tools

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python](https://img.shields.io/badge/Python-3.12-blue)](https://www.python.org/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-orange)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

Utilities, tools, and reference implementations from the **devibot AMR platform** — Peribott Dynamic LLP's production-grade autonomous mobile robot built on ROS2 Jazzy.

> This repository contains the open-source tooling layer of devibot. The proprietary firmware, navigation stack configuration, and fleet management platform are not included.

---

## Platform Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    devibot AMR Stack                        │
├─────────────────────────────────────────────────────────────┤
│  Layer 6 │ Cloud Fleet Platform  (Docker · MongoDB · WS)    │
│  Layer 5 │ AMR Dashboard         (FastAPI · React/TS)       │
│  Layer 4 │ ROS2 Navigation       (Nav2 · SLAM · AMCL)       │
│  Layer 3 │ Sensor Integration    (LD19 LiDAR · IMU · US)    │
│  Layer 2 │ Power Management      (Pylontech BMS · CAN)      │
│  Layer 1 │ Motor Control         (STM32G491 · CANopen 402)  │
└─────────────────────────────────────────────────────────────┘
```

**Hardware**
- MCU: STM32G491 · FreeRTOS · FDCAN
- Motor Drives: Syntron DS20230C · CANopen 402
- Navigation: ROS2 Jazzy · Nav2 · SLAM Toolbox
- Sensors: LD19 LiDAR · IMU · 3× Ultrasonic
- BMS: Pylontech · CAN bus · Charge safety guards
- Dashboard: FastAPI + React/TypeScript · 24+ ROS2 topics
- Boot System: v4.2.6 · Ubuntu 24.04 · <90s to operational

---

## Repository Structure

```
devibot-ros2-tools/
├── qos_profiles/
│   ├── __init__.py
│   └── peribott_qos.py          # Shared QoS profiles for all devibot nodes
├── packet_decoder/
│   ├── __init__.py
│   └── devibot_packet_decoder.py # STM32→ROS2 UART packet decoder
├── scripts/
│   ├── ros2_health_check.sh     # Node/topic health verification
│   ├── check_can_bus.sh         # CANopen bus diagnostics
│   └── log_viewer.sh            # Structured log viewer with filtering
├── launch/
│   └── diagnostics.launch.py    # Diagnostics-only launch file
├── config/
│   └── qos_overrides.yaml       # Runtime QoS override configuration
├── examples/
│   ├── qos_publisher_example.py # How to use peribott_qos in a node
│   ├── packet_decoder_demo.py   # Decode live STM32 UART stream
│   └── health_monitor.py        # Standalone node health monitor
├── docs/
│   ├── system_architecture.md   # Full platform architecture
│   ├── can_bus_setup.md         # CANopen + FDCAN configuration guide
│   ├── qos_design.md            # QoS decision rationale
│   └── troubleshooting.md       # Common issues and solutions
├── tests/
│   ├── test_packet_decoder.py   # Unit tests for packet decoder
│   └── test_qos_profiles.py     # QoS profile validation tests
├── LICENSE
└── README.md
```

---

## Installation

### Prerequisites

```bash
# Ubuntu 24.04 LTS
sudo apt update && sudo apt install -y \
    python3-pip \
    ros-jazzy-ros-base \
    python3-rclpy \
    can-utils \
    python3-pytest

source /opt/ros/jazzy/setup.bash
```

### Install

```bash
git clone https://github.com/peribott/devibot-ros2-tools.git
cd devibot-ros2-tools
pip3 install -r requirements.txt --break-system-packages
```

---

## Modules

### `qos_profiles` — Shared QoS Profiles

Centralised QoS definitions used across all devibot ROS2 nodes. Importing from a single source guarantees publisher/subscriber compatibility across the entire system.

```python
from qos_profiles.peribott_qos import LATCHED_QOS, SENSOR_QOS, CMD_QOS

# Publisher — shutdown request (latched, never missed by late subscribers)
self.shutdown_pub = self.create_publisher(String, '/robot/shutdown_request', LATCHED_QOS)

# Subscriber — sensor data (best effort, high frequency)
self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, SENSOR_QOS)
```

See [docs/qos_design.md](docs/qos_design.md) for the rationale behind each profile.

---

### `packet_decoder` — STM32 UART Packet Decoder

Decodes the binary packet protocol used for STM32↔ROS2 communication over UART. Covers all packet types: motor telemetry, BMS state, health data, navigation safety, encoder feedback.

```python
from packet_decoder.devibot_packet_decoder import DevibotPacketDecoder

decoder = DevibotPacketDecoder()

# Feed raw bytes from UART
decoder.feed(raw_bytes)

# Get decoded packets
for packet in decoder.get_packets():
    print(packet.packet_type, packet.data)
```

See [examples/packet_decoder_demo.py](examples/packet_decoder_demo.py) for a complete live UART demo.

---

### `scripts/` — Diagnostic Scripts

| Script | Purpose |
|--------|---------|
| `ros2_health_check.sh` | Verify all required nodes and topics are alive |
| `check_can_bus.sh` | Check CANopen bus status, list active CAN IDs |
| `log_viewer.sh` | Tail and filter rotating log files with colour |

```bash
# Check all devibot nodes are running
./scripts/ros2_health_check.sh

# Diagnose CAN bus — requires can-utils
./scripts/check_can_bus.sh can0

# View logs with error filter
./scripts/log_viewer.sh --filter ERROR
```

---

## Key Design Decisions

### Why RELIABLE + TRANSIENT_LOCAL for control topics?

Default ROS2 QoS (RELIABLE + VOLATILE) drops messages for subscribers that join after publication. For latched topics — shutdown requests, configuration, sync events — this causes silent failures. See [docs/qos_design.md](docs/qos_design.md).

### Why a binary packet protocol over UART?

JSON over UART is human-readable but slow and fragile: a single corrupt byte invalidates the entire frame. The devibot binary protocol uses fixed-length packets with CRC16 validation, runs at 460800 baud, and delivers <2ms latency for motor telemetry. See [docs/system_architecture.md](docs/system_architecture.md).

### Why NaN sanitisation before JSON serialisation?

The FastAPI→WebSocket pipeline serialises ROS2 topic data to JSON. Ultrasonic sensors return `float('nan')` on no-echo conditions. JSON does not support NaN — it causes a serialisation exception that drops the WebSocket connection. The decoder sanitises all floats before they reach the serialisation layer.

---

## Testing

```bash
cd devibot-ros2-tools
python3 -m pytest tests/ -v
```

---

## Related

- **Website**: [www.jagnani.com](https://www.jagnani.com)
- **Company**: [peribott.com](https://peribott.com)
- **Articles**: [Why Most AMRs Fail in Production](https://www.jagnani.com/articles/amr-production-failure.html)
- **Contact**: amit@peribott.com

---

## License

MIT License — see [LICENSE](LICENSE).

The devibot AMR platform firmware, dashboard, and fleet management system are proprietary and not included in this repository.

---

*Built by [Amit Jagnani](https://www.jagnani.com) · Peribott Dynamic LLP · Hyderabad, India*
