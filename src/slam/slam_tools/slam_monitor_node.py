#!/usr/bin/env python3

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class SlamMonitorNode(Node):
    """
    Simple SLAM monitoring node.

    Subscribes to /map (OccupancyGrid) and periodically logs:
    - map width, height, resolution
    - occupancy ratio (occupied cells / total known cells)
    - update rate (Hz)
    """

    def __init__(self) -> None:
        super().__init__("slam_monitor")

        # Subscribe to the occupancy grid map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            10,
        )

        self.last_map: Optional[OccupancyGrid] = None
        self.map_msg_count: int = 0
        self.last_report_time: float = time.time()

        # How often to print summaries (seconds)
        self.report_interval: float = 2.0

        self.get_logger().info("SLAM monitor node started, listening on /map")

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.last_map = msg
        self.map_msg_count += 1

        now = time.time()
        if (now - self.last_report_time) >= self.report_interval:
            self.report_status(now)

    def report_status(self, now: float) -> None:
        if self.last_map is None:
            return

        # Compute update rate
        dt = now - self.last_report_time
        hz = self.map_msg_count / dt if dt > 0.0 else 0.0

        info = self.last_map.info
        width = info.width
        height = info.height
        resolution = info.resolution

        data = self.last_map.data

        known_cells = [v for v in data if v != -1]
        occupied_cells = [v for v in data if v > 50]  # threshold for "occupied"

        known_count = len(known_cells)
        occupied_count = len(occupied_cells)

        occupancy_ratio = (
            float(occupied_count) / known_count if known_count > 0 else 0.0
        )

        self.get_logger().info(
            f"SLAM status: "
            f"map={width}x{height} res={resolution:.3f} m, "
            f"update_rate={hz:.2f} Hz, "
            f"occupancy={occupancy_ratio*100:.1f}% of known cells"
        )

        # Reset counters
        self.map_msg_count = 0
        self.last_report_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
