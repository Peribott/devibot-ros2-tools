"""
health_monitor.py
------------------
Standalone ROS2 node that monitors devibot system health and
prints a live status table to the terminal.

Run:
    source /opt/ros/jazzy/setup.bash
    python3 examples/health_monitor.py

Press Ctrl+C to stop.

Peribott Dynamic LLP — Hyderabad, India
"""

import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from qos_profiles.peribott_qos import LATCHED_QOS, SENSOR_QOS, STATUS_QOS


class HealthMonitor(Node):
    """
    Subscribes to key devibot topics and prints a live status dashboard.
    """

    def __init__(self) -> None:
        super().__init__("devibot_health_monitor")

        # State
        self._battery_pct: float = 0.0
        self._battery_voltage: float = 0.0
        self._odom_linear: float = 0.0
        self._odom_angular: float = 0.0
        self._scan_min_range: float = float("inf")
        self._robot_status: str = "UNKNOWN"
        self._nav_state: str = "UNKNOWN"
        self._last_scan_ts: float = 0.0
        self._last_odom_ts: float = 0.0
        self._last_battery_ts: float = 0.0
        self._start_time: float = time.monotonic()

        # Subscriptions
        self.create_subscription(
            BatteryState, "/battery_state", self._on_battery, SENSOR_QOS
        )
        self.create_subscription(
            Odometry, "/odom", self._on_odom, SENSOR_QOS
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan, SENSOR_QOS
        )
        self.create_subscription(
            String, "/robot/status", self._on_status, STATUS_QOS
        )
        self.create_subscription(
            String, "/robot/nav_state", self._on_nav_state, STATUS_QOS
        )

        # Refresh terminal every second
        self.create_timer(1.0, self._print_status)
        self.get_logger().info("Health monitor started")

    def _on_battery(self, msg: BatteryState) -> None:
        self._battery_pct = msg.percentage * 100.0
        self._battery_voltage = msg.voltage
        self._last_battery_ts = time.monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_linear = msg.twist.twist.linear.x
        self._odom_angular = msg.twist.twist.angular.z
        self._last_odom_ts = time.monotonic()

    def _on_scan(self, msg: LaserScan) -> None:
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        self._scan_min_range = min(valid) if valid else float("inf")
        self._last_scan_ts = time.monotonic()

    def _on_status(self, msg: String) -> None:
        self._robot_status = msg.data

    def _on_nav_state(self, msg: String) -> None:
        self._nav_state = msg.data

    def _age(self, ts: float) -> str:
        if ts == 0.0:
            return "never"
        age = time.monotonic() - ts
        if age < 2.0:
            return "live"
        return f"{age:.0f}s ago"

    def _bat_bar(self, pct: float, width: int = 10) -> str:
        filled = int(pct / 100.0 * width)
        bar = "█" * filled + "░" * (width - filled)
        return f"[{bar}] {pct:.0f}%"

    def _print_status(self) -> None:
        uptime = time.monotonic() - self._start_time
        h = int(uptime // 3600)
        m = int((uptime % 3600) // 60)
        s = int(uptime % 60)

        # Clear screen and move to top
        print("\033[2J\033[H", end="")

        print("╔═══════════════════════════════════════════════════╗")
        print("║        devibot AMR  —  Health Monitor             ║")
        print(f"║  Monitor uptime: {h:02d}:{m:02d}:{s:02d}                          ║")
        print("╠═══════════════════════════════════════════════════╣")
        print(f"║  Robot status : {self._robot_status:<33}║")
        print(f"║  Nav state    : {self._nav_state:<33}║")
        print("╠═══════════════════════════════════════════════════╣")
        print(f"║  Battery      : {self._bat_bar(self._battery_pct):<33}║")
        print(f"║  Voltage      : {self._battery_voltage:<6.2f}V   ({self._age(self._last_battery_ts):<20})║")
        print("╠═══════════════════════════════════════════════════╣")
        print(f"║  Linear vel   : {self._odom_linear:+.3f} m/s  ({self._age(self._last_odom_ts):<19})║")
        print(f"║  Angular vel  : {self._odom_angular:+.3f} rad/s({self._age(self._last_odom_ts):<19})║")
        print("╠═══════════════════════════════════════════════════╣")
        scan_str = f"{self._scan_min_range:.2f}m" if self._scan_min_range < float("inf") else "no echo"
        print(f"║  LiDAR min    : {scan_str:<14} ({self._age(self._last_scan_ts):<19})║")
        print("╚═══════════════════════════════════════════════════╝")
        print("\n  Press Ctrl+C to stop.")


def main() -> None:
    rclpy.init()
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nHealth monitor stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
