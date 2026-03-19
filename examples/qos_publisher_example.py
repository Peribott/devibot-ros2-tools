"""
qos_publisher_example.py
-------------------------
Demonstrates the correct way to use peribott_qos profiles
in a ROS2 node.

Run:
    python3 examples/qos_publisher_example.py

Peribott Dynamic LLP — Hyderabad, India
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from sensor_msgs.msg import LaserScan

# Import shared QoS profiles
from qos_profiles.peribott_qos import (
    LATCHED_QOS,
    SENSOR_QOS,
    CMD_QOS,
    STATUS_QOS,
    describe_qos,
)


class ExampleNode(Node):
    """
    Demonstrates correct QoS profile usage for different topic categories.
    """

    def __init__(self) -> None:
        super().__init__("devibot_example_node")

        # ── Latched publisher: shutdown request ───────────────────────────────
        # Use LATCHED_QOS for any topic where late-joining subscribers
        # must receive the current value immediately on connect.
        # Example: the dashboard subscribes after the shutdown was requested —
        # it must still receive the message.
        self.shutdown_pub = self.create_publisher(
            String,
            "/robot/shutdown_request",
            LATCHED_QOS,
        )

        # ── Latched publisher: robot configuration ────────────────────────────
        self.config_pub = self.create_publisher(
            String,
            "/robot/config",
            LATCHED_QOS,
        )

        # ── Sensor subscriber: LiDAR scan ─────────────────────────────────────
        # Use SENSOR_QOS (BEST_EFFORT + VOLATILE) for high-frequency sensor data.
        # Dropped frames are acceptable — stale data is not.
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            SENSOR_QOS,
        )

        # ── Status publisher: robot nav state ─────────────────────────────────
        self.status_pub = self.create_publisher(
            String,
            "/robot/nav_state",
            STATUS_QOS,
        )

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(2.0, self._publish_status)
        self.create_timer(10.0, self._publish_config)

        self.get_logger().info("Example node started")
        self._log_qos_info()

    def _log_qos_info(self) -> None:
        """Log the QoS profiles in use — useful for debugging mismatches."""
        self.get_logger().info(
            "QoS profiles:\n"
            "  /robot/shutdown_request : %s\n"
            "  /scan                   : %s\n"
            "  /robot/nav_state        : %s",
            describe_qos(LATCHED_QOS),
            describe_qos(SENSOR_QOS),
            describe_qos(STATUS_QOS),
        )

    def _on_scan(self, msg: LaserScan) -> None:
        """Process incoming LiDAR scan. Called at sensor rate (~10 Hz)."""
        if msg.ranges:
            min_range = min(r for r in msg.ranges if r > msg.range_min)
            self.get_logger().debug("Closest obstacle: %.2f m", min_range)

    def _publish_status(self) -> None:
        msg = String()
        msg.data = "NAVIGATING"
        self.status_pub.publish(msg)

    def _publish_config(self) -> None:
        """
        Publish robot configuration as a latched topic.
        Any subscriber that connects later will immediately receive this.
        """
        msg = String()
        msg.data = '{"map": "warehouse_floor_1", "home": [0.0, 0.0, 0.0]}'
        self.config_pub.publish(msg)
        self.get_logger().info("Published config (latched)")


def main() -> None:
    rclpy.init()
    node = ExampleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
