# devibot System Architecture

## Overview

devibot is a production-grade autonomous mobile robot built by Peribott Dynamic LLP.
The architecture is designed around one principle: **fail safely and visibly,
never silently**.

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | STM32G491RET6 | — |
| RTOS | FreeRTOS 10.4 | — |
| Motor drives | Syntron DS20230C (×2) | FDCAN / CANopen 402 |
| BMS | Pylontech | CAN (extended 29-bit) |
| LiDAR | LDROBOT LD19 | UART / USB |
| IMU | MPU-6050 | I²C |
| Ultrasonics | HC-SR04 (×3) | GPIO / Timer |
| Compute | Intel NUC (Ubuntu 24.04) | USB / UART |

---

## Software Stack

```
┌──────────────────────────────────────────────────────────────────┐
│  Cloud Fleet Platform                                            │
│  FastAPI (Python) + React/TypeScript + Socket.IO                 │
│  MongoDB (telemetry) + PostgreSQL (config) + Redis (cache)       │
├──────────────────────────────────────────────────────────────────┤
│  AMR Dashboard (on-robot kiosk)                                  │
│  FastAPI backend + React 18/TypeScript/Tailwind frontend         │
│  Subscriptions: 24+ ROS2 topics via WebSocket bridge             │
├──────────────────────────────────────────────────────────────────┤
│  ROS2 Jazzy Layer                                                │
│  ┌─────────────┐ ┌──────────────┐ ┌─────────────┐              │
│  │ devibot_    │ │ map_manager  │ │ cloud_bridge│              │
│  │ bridge      │ │              │ │             │              │
│  └──────┬──────┘ └──────┬───────┘ └──────┬──────┘              │
│         │               │                │                       │
│  ┌──────▼───────────────▼────────────────▼──────┐              │
│  │  Nav2 Stack                                   │              │
│  │  BT Navigator · Planner · Controller          │              │
│  │  Map Server · AMCL · Costmaps                 │              │
│  └──────────────────────────────────────────────┘              │
│  ┌────────────────────────────────────────────────┐             │
│  │  SLAM Toolbox                                  │             │
│  └────────────────────────────────────────────────┘             │
├──────────────────────────────────────────────────────────────────┤
│  STM32 Firmware Layer                                            │
│  FreeRTOS tasks:                                                 │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐          │
│  │ CAN Rx   │ │ CAN Tx   │ │ UART Tx  │ │ UART Rx  │          │
│  │ Task     │ │ Task     │ │ Task     │ │ Task     │          │
│  └──────────┘ └──────────┘ └──────────┘ └──────────┘          │
│  ┌──────────┐ ┌──────────┐ ┌──────────┐                        │
│  │ Motion   │ │ Sensor   │ │ Health   │                        │
│  │ Control  │ │ Sample   │ │ Monitor  │                        │
│  └──────────┘ └──────────┘ └──────────┘                        │
├──────────────────────────────────────────────────────────────────┤
│  Hardware                                                        │
│  CANopen Bus ─── Motor ×2 ─── Encoder ×2 ─── BMS              │
│  UART ──────────────────────────────── (STM32 ↔ NUC)           │
│  Sensors: LiDAR + IMU + Ultrasonic ×3                           │
└──────────────────────────────────────────────────────────────────┘
```

---

## STM32 ↔ ROS2 Communication

The STM32 and the ROS2 host (Intel NUC) communicate via UART at **460800 baud**
using a custom binary packet protocol. See [packet_decoder/devibot_packet_decoder.py](../packet_decoder/devibot_packet_decoder.py)
for the full protocol specification.

**Why binary over UART instead of JSON or rosserial?**

| Criterion | JSON/UART | Binary Protocol |
|-----------|-----------|-----------------|
| Latency | ~5ms (parse overhead) | <2ms |
| Frame corruption | Silently produces bad values | CRC16 rejects corrupt frames |
| Bandwidth | ~3× larger | Compact fixed-length packets |
| NaN safety | NaN becomes "null" in JSON | Sanitised at decode |
| Debugging | Human-readable | Requires decoder (provided) |

---

## ROS2 Node Architecture

### `devibot_bridge`
- Reads binary packets from `/dev/ttyS0` at 460800 baud
- Decodes via `DevibotPacketDecoder`
- Publishes to ROS2 topics:
  - `/battery_state` (BatteryState) — from MSG_BMS_STATE
  - `/wheel_encoders` (custom) — from MSG_ENCODER_DATA
  - `/ultrasonic/front_left`, `/front_right`, `/rear` (Range)
  - `/robot/health` (DiagnosticStatus) — from MSG_HEALTH_DATA
  - `/robot/nav_safety` (custom) — from MSG_NAV_SAFETY
- Subscribes to `/cmd_vel` and forwards motor commands via CAN over STM32
- Handles heartbeat exchange — bridge sends ping, STM32 must respond within 2s

### `map_manager`
- Manages SLAM map lifecycle: save, load, delete, list
- Publishes cloud sync events via `/cloud/sync_event` (CLOUD_SYNC_QOS)
- Services: `/map_manager/save_map`, `/load_map`, `/delete_map`, `/list_maps`
- Prevents map name collisions with real-time validation

### `cloud_bridge`
- Maintains WebSocket connection to fleet management cloud platform
- Streams telemetry at configurable rate (default 5 Hz)
- Handles map sync, config push, and remote command reception
- Automatic reconnect with exponential backoff

---

## Boot Sequence (AMR Boot System v4.2.6)

```
Power On
   │
   ▼
Phase 1: Backend Services
   │  Start robot-dashboard.service
   │  Wait for FastAPI health endpoint: GET /health → 200
   │
   ▼
Phase 2: Cloud Connectivity
   │  Verify network interface up
   │  Attempt cloud WebSocket handshake (timeout: 10s, non-fatal)
   │
   ▼
Phase 3: SLAM & Navigation Stack
   │  ros2 launch devibot_nav navigation.launch.py
   │  Health check: ros2 node list | grep nav2_bt_navigator
   │  Health check: ros2 node list | grep slam_toolbox
   │
   ▼
Phase 4: ROS2 Bridge
   │  Start devibot_bridge node
   │  Verify /battery_state topic publishing (timeout: 5s)
   │  Verify heartbeat exchange with STM32 (timeout: 3s)
   │
   ▼
Phase 5: Dashboard UI
   │  All FastAPI routes verified
   │  WebSocket bridge connected
   │
   ▼
Phase 6: Kiosk Browser
   │  Dismiss Plymouth splash
   │  Launch Firefox ESR in kiosk mode → localhost:8080
   │
   ▼
OPERATIONAL (< 90 seconds from power-on)
```

**Safe Mode** — if any phase fails after 3 retries:
- Navigation stack is stopped (safety)
- Motors are disabled
- Diagnostics dashboard launched at localhost:8081
- Plymouth replaced with "Safe Mode" screen
- Failure reason logged to `/var/log/robot-dashboard/boot.log`

---

## Key Design Decisions

### NaN sanitisation

All float values decoded from STM32 packets pass through `_sanitise_float()` before
reaching ROS2 or JSON serialisation. Ultrasonic sensors return IEEE 754 NaN on no-echo.
JSON does not support NaN — one unsanitised NaN drops the entire WebSocket connection.

### QoS profiles

See [qos_profiles/peribott_qos.py](../qos_profiles/peribott_qos.py) for the full rationale.
Short version: RELIABLE + TRANSIENT_LOCAL for anything published once (config, shutdown, sync events).
BEST_EFFORT + VOLATILE for sensor data. Never mix these at publisher/subscriber.

### Motor-stop-first shutdown

On any shutdown trigger (button hold, ROS2 request, watchdog, fault):
1. Motor stop command sent via CAN
2. Navigation stack cancelled
3. ROS2 nodes signalled
4. System shutdown

This ordering prevents the robot from continuing to move during shutdown.

---

*Peribott Dynamic LLP · Hyderabad, India · [peribott.com](https://peribott.com)*
