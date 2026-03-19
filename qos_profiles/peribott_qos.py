"""
peribott_qos.py
---------------
Shared ROS2 QoS profiles for all devibot nodes.

Importing from a single source guarantees that publisher and subscriber
QoS profiles are always compatible. A QoS mismatch in ROS2 does not
raise an error — the connection silently fails or degrades.

Usage:
    from qos_profiles.peribott_qos import LATCHED_QOS, SENSOR_QOS, CMD_QOS

Peribott Dynamic LLP — Hyderabad, India
https://peribott.com
"""

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
)
from rclpy.duration import Duration


# ── LATCHED_QOS ──────────────────────────────────────────────────────────────
#
# Use for: shutdown requests, configuration topics, licence status,
#          cloud sync events, any topic where late-joining subscribers
#          must receive the most recent value immediately on connect.
#
# RELIABLE   → retransmit until acknowledged — no silent drops
# TRANSIENT_LOCAL → publisher stores last `depth` messages; new subscribers
#                   receive stored messages even if published before they existed
#
# Both publisher AND subscriber must use this profile. A TRANSIENT_LOCAL
# publisher + VOLATILE subscriber will connect but the subscriber will NOT
# receive stored messages.
#
LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── SENSOR_QOS ───────────────────────────────────────────────────────────────
#
# Use for: LiDAR scans, IMU data, ultrasonic ranges, odometry,
#          camera images — any high-frequency sensor topic.
#
# BEST_EFFORT → drop frames rather than queue; always deliver the freshest data
# VOLATILE    → no message storage; subscribers receive only future messages
# depth=5     → small buffer to absorb brief processing spikes
#
# Do NOT use RELIABLE here — retransmission causes latency accumulation
# that makes old sensor data appear newer than it is. For navigation,
# stale sensor data is worse than no data.
#
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# ── CMD_QOS ──────────────────────────────────────────────────────────────────
#
# Use for: velocity commands (/cmd_vel), navigation goals,
#          waypoint commands, e-stop signals.
#
# RELIABLE → delivery guaranteed — a dropped velocity command is a safety issue
# VOLATILE → no storage; commands are only valid at the moment of sending
# depth=10 → queue commands during brief processing delays
#
CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── STATUS_QOS ───────────────────────────────────────────────────────────────
#
# Use for: robot_status, navigation_state, error_state —
#          periodic status broadcasts where occasional drops are acceptable
#          but the subscriber should always have a recent value.
#
# RELIABLE + KEEP_LAST(1): always deliver, always fresh.
# Suitable for topics published at 1-5 Hz.
#
STATUS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── MAP_QOS ───────────────────────────────────────────────────────────────────
#
# Use for: /map, /global_costmap/costmap, map metadata.
#
# RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1):
# Map topics are large and published infrequently. Late subscribers
# (e.g. RViz starting after SLAM) must receive the current map immediately.
# depth=1: only the latest map is meaningful.
#
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# ── CLOUD_SYNC_QOS ────────────────────────────────────────────────────────────
#
# Use for: /cloud/sync_event — event-driven cloud synchronisation.
#
# Events must never be dropped (RELIABLE) and must be received by any
# node that connects after the event was published (TRANSIENT_LOCAL).
# Stores last 10 events to handle brief connectivity gaps.
#
CLOUD_SYNC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── DIAGNOSTICS_QOS ───────────────────────────────────────────────────────────
#
# Use for: /diagnostics, /diagnostics_agg — standard ROS2 diagnostics.
# Matches the expected profile of the diagnostics_aggregator.
#
DIAGNOSTICS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# ── PROFILE REGISTRY ─────────────────────────────────────────────────────────
#
# Map topic patterns to their expected QoS profile.
# Used by the health monitor to verify running nodes use correct profiles.
#
TOPIC_QOS_MAP: dict[str, QoSProfile] = {
    "/robot/shutdown_request":      LATCHED_QOS,
    "/robot/shutdown_command":      LATCHED_QOS,
    "/robot/config":                LATCHED_QOS,
    "/robot/licence_status":        LATCHED_QOS,
    "/cloud/sync_event":            CLOUD_SYNC_QOS,
    "/scan":                        SENSOR_QOS,
    "/imu/data":                    SENSOR_QOS,
    "/ultrasonic/front_left":       SENSOR_QOS,
    "/ultrasonic/front_right":      SENSOR_QOS,
    "/ultrasonic/rear":             SENSOR_QOS,
    "/odom":                        SENSOR_QOS,
    "/cmd_vel":                     CMD_QOS,
    "/map":                         MAP_QOS,
    "/global_costmap/costmap":      MAP_QOS,
    "/robot/status":                STATUS_QOS,
    "/robot/nav_state":             STATUS_QOS,
    "/diagnostics":                 DIAGNOSTICS_QOS,
}


def get_qos_for_topic(topic_name: str) -> QoSProfile:
    """
    Return the appropriate QoS profile for a given topic name.
    Falls back to CMD_QOS (reliable, volatile) for unknown topics.

    Args:
        topic_name: ROS2 topic name (e.g. '/scan', '/cmd_vel')

    Returns:
        QoSProfile matching the topic's expected communication pattern
    """
    return TOPIC_QOS_MAP.get(topic_name, CMD_QOS)


def describe_qos(profile: QoSProfile) -> str:
    """Return a human-readable description of a QoS profile."""
    reliability = profile.reliability.name
    durability = profile.durability.name
    depth = profile.depth
    return f"{reliability} + {durability} (depth={depth})"
