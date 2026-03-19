# ROS2 QoS Design — devibot

## The Problem

Default ROS2 QoS (RELIABLE + VOLATILE) silently drops messages
for subscribers that join after publication. This caused a specific
production bug in devibot: shutdown requests were occasionally missed
by the dashboard if it was still starting up when the request arrived.

## Profile Selection Guide

| Use case | Profile | Why |
|----------|---------|-----|
| Shutdown / reboot requests | `LATCHED_QOS` | Must never be missed, even by late joiners |
| Robot configuration | `LATCHED_QOS` | Config published once; subscribers may join any time |
| Cloud sync events | `CLOUD_SYNC_QOS` | Events must be delivered; store last 10 for recovery |
| LiDAR scan | `SENSOR_QOS` | Freshness over completeness; dropped frames OK |
| Odometry | `SENSOR_QOS` | High frequency; retransmission causes latency |
| cmd_vel | `CMD_QOS` | Reliability matters; stale velocity = safety risk |
| Map topics | `MAP_QOS` | Large, infrequent; late subscribers need current map |
| Robot status | `STATUS_QOS` | Periodic; always deliver latest |

## Common Mistake

A TRANSIENT_LOCAL publisher and a VOLATILE subscriber will
**connect without error** but the subscriber will **not receive
stored messages**. Both sides must use TRANSIENT_LOCAL.

Check mismatches with:
```bash
ros2 topic info /your_topic --verbose
```

## Reference

- [peribott_qos.py](../qos_profiles/peribott_qos.py) — implementation
- [ROS2 QoS docs](https://docs.ros.org/en/jazzy/Concepts/About-Quality-of-Service-Settings.html)

---
*Peribott Dynamic LLP · [peribott.com](https://peribott.com)*
