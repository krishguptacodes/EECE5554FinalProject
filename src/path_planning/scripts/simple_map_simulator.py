#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header


class SimpleMapSimulator(Node):
    def __init__(self):
        super().__init__('simple_map_simulator')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('map_width', 100),      # cells
                ('map_height', 100),     # cells
                ('resolution', 0.1),     # meters/cell
                ('origin_x', -5.0),      # meters
                ('origin_y', -5.0),      # meters
                ('publish_rate', 1.0),   # Hz
                ('map_type', 'room_with_obstacles'),  # room_with_obstacles, maze, empty, office
            ])

        # Get parameters
        self.map_topic = self.get_parameter('map_topic').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_type = self.get_parameter('map_type').value

        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_map)

        # Generate map
        self.map_data = self.generate_map()

        self.get_logger().info(f"Simple Map Simulator started - {self.map_type}")
        self.get_logger().info(f"Map size: {self.map_width}x{self.map_height}, resolution: {self.resolution}m")

    def generate_map(self):
        """Generate different types of test maps"""
        if self.map_type == 'empty':
            return self.create_empty_map()
        elif self.map_type == 'room_with_obstacles':
            return self.create_room_with_obstacles()
        elif self.map_type == 'maze':
            return self.create_simple_maze()
        elif self.map_type == 'office':
            return self.create_office_layout()
        else:
            self.get_logger().warn(f"Unknown map type: {self.map_type}, using empty map")
            return self.create_empty_map()

    def create_empty_map(self):
        """Create an empty map (all free space)"""
        grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Add border walls
        grid[0, :] = 100  # Top wall
        grid[-1, :] = 100  # Bottom wall
        grid[:, 0] = 100  # Left wall
        grid[:, -1] = 100  # Right wall

        return grid

    def create_room_with_obstacles(self):
        """Create a room with some obstacles"""
        grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Add border walls
        grid[0, :] = 100  # Top wall
        grid[-1, :] = 100  # Bottom wall
        grid[:, 0] = 100  # Left wall
        grid[:, -1] = 100  # Right wall

        # Add some rectangular obstacles
        # Obstacle 1: Small rectangle
        grid[20:30, 20:35] = 100

        # Obstacle 2: L-shaped obstacle
        grid[50:70, 60:80] = 100
        grid[35:55, 75:80] = 100

        # Obstacle 3: Circular-ish obstacle
        center_x, center_y = 30, 70
        radius = 8
        for y in range(max(0, center_y - radius), min(self.map_height, center_y + radius)):
            for x in range(max(0, center_x - radius), min(self.map_width, center_x + radius)):
                if (x - center_x)**2 + (y - center_y)**2 <= radius**2:
                    grid[y, x] = 100

        # Obstacle 4: Narrow corridor
        grid[10:15, 50:90] = 100
        grid[10:40, 70:75] = 100

        return grid

    def create_simple_maze(self):
        """Create a simple maze"""
        grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Fill with walls
        grid.fill(100)

        # Create corridors
        # Horizontal corridors
        grid[20, 10:90] = 0
        grid[40, 10:90] = 0
        grid[60, 10:90] = 0
        grid[80, 10:90] = 0

        # Vertical corridors
        grid[10:90, 20] = 0
        grid[10:90, 40] = 0
        grid[10:90, 60] = 0
        grid[10:90, 80] = 0

        # Create openings in walls
        grid[15:25, 30] = 0  # Opening 1
        grid[35:45, 50] = 0  # Opening 2
        grid[55:65, 70] = 0  # Opening 3
        grid[30, 25:35] = 0  # Opening 4
        grid[50, 45:55] = 0  # Opening 5
        grid[70, 65:75] = 0  # Opening 6

        return grid

    def create_office_layout(self):
        """Create an office-like layout"""
        grid = np.zeros((self.map_height, self.map_width), dtype=np.int8)

        # Add border walls
        grid[0, :] = 100  # Top wall
        grid[-1, :] = 100  # Bottom wall
        grid[:, 0] = 100  # Left wall
        grid[:, -1] = 100  # Right wall

        # Add office rooms
        # Room 1
        grid[10:30, 10:40] = 100
        grid[15:25, 15:35] = 0  # Interior
        grid[20, 10] = 0  # Door

        # Room 2
        grid[10:30, 60:90] = 100
        grid[15:25, 65:85] = 0  # Interior
        grid[20, 90] = 0  # Door

        # Room 3
        grid[40:60, 10:40] = 100
        grid[45:55, 15:35] = 0  # Interior
        grid[50, 10] = 0  # Door

        # Room 4
        grid[40:60, 60:90] = 100
        grid[45:55, 65:85] = 0  # Interior
        grid[50, 90] = 0  # Door

        # Central corridor
        grid[30:40, 10:90] = 0
        grid[10:90, 45:55] = 0

        # Add some furniture/obstacles
        grid[35:37, 25:30] = 100  # Table
        grid[35:37, 65:70] = 100  # Table

        return grid

    def create_map_message(self):
        """Create ROS OccupancyGrid message"""
        msg = OccupancyGrid()

        # Header
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        # Map metadata
        msg.info = MapMetaData()
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.resolution = self.resolution

        # Origin pose
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Map data (flatten row-major)
        msg.data = self.map_data.flatten().tolist()

        return msg

    def publish_map(self):
        """Publish the map"""
        map_msg = self.create_map_message()
        self.map_pub.publish(map_msg)

        # Log statistics
        total_cells = self.map_width * self.map_height
        occupied_cells = np.sum(self.map_data == 100)
        free_cells = np.sum(self.map_data == 0)
        unknown_cells = total_cells - occupied_cells - free_cells

        self.get_logger().info(
            f"Published {self.map_type} map: "
            f"Free: {free_cells}, Occupied: {occupied_cells}, Unknown: {unknown_cells}"
        )

    def add_dynamic_obstacle(self, x, y, radius=3):
        """Add a dynamic obstacle at world coordinates"""
        # Convert world coordinates to grid
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)

        # Add circular obstacle
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    gx, gy = grid_x + dx, grid_y + dy
                    if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                        self.map_data[gy, gx] = 100

        self.get_logger().info(f"Added dynamic obstacle at ({x:.1f}, {y:.1f})")

    def clear_dynamic_obstacles(self):
        """Clear all dynamic obstacles and regenerate base map"""
        self.map_data = self.generate_map()
        self.get_logger().info("Cleared all dynamic obstacles")


def main(args=None):
    rclpy.init(args=args)

    simulator = SimpleMapSimulator()

    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("Map simulator shutting down")
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()