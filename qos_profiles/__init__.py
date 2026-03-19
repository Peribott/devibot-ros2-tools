"""
qos_profiles — Shared ROS2 QoS profiles for devibot nodes.
Peribott Dynamic LLP · Hyderabad, India
"""

from .peribott_qos import (
    LATCHED_QOS,
    SENSOR_QOS,
    CMD_QOS,
    STATUS_QOS,
    MAP_QOS,
    CLOUD_SYNC_QOS,
    DIAGNOSTICS_QOS,
    TOPIC_QOS_MAP,
    get_qos_for_topic,
    describe_qos,
)

__all__ = [
    "LATCHED_QOS",
    "SENSOR_QOS",
    "CMD_QOS",
    "STATUS_QOS",
    "MAP_QOS",
    "CLOUD_SYNC_QOS",
    "DIAGNOSTICS_QOS",
    "TOPIC_QOS_MAP",
    "get_qos_for_topic",
    "describe_qos",
]
