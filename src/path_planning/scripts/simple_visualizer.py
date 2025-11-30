#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped


class SimpleVisualizer(Node):
    def __init__(self):
        super().__init__('simple_visualizer')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('path_topic', '/path'),
                ('update_rate', 2.0),  # Hz
            ])

        self.map_topic = self.get_parameter('map_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.update_rate = self.get_parameter('update_rate').value

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )

        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            10
        )

        # Data storage
        self.map_data = None
        self.map_info = None
        self.path_data = None
        self.data_lock = threading.Lock()

        # Matplotlib setup
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_title('Path Planning Visualization')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')

        # Animation
        self.animation = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=int(1000 / self.update_rate),
            blit=False
        )

        self.get_logger().info("Simple Visualizer initialized")

    def map_callback(self, msg):
        """Process incoming map data"""
        with self.data_lock:
            # Convert map data to 2D array
            self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            self.map_info = msg.info

        self.get_logger().info(f"Received map: {msg.info.width}x{msg.info.height}")

    def path_callback(self, msg):
        """Process incoming path data"""
        with self.data_lock:
            if len(msg.poses) > 0:
                self.path_data = [(pose.pose.position.x, pose.pose.position.y)
                                for pose in msg.poses]
            else:
                self.path_data = None

        self.get_logger().info(f"Received path with {len(msg.poses)} points")

    def update_plot(self, frame):
        """Update the visualization"""
        with self.data_lock:
            # Clear the plot
            self.ax.clear()
            self.ax.set_title('Path Planning Visualization')
            self.ax.set_xlabel('X (meters)')
            self.ax.set_ylabel('Y (meters)')
            self.ax.grid(True, alpha=0.3)

            # Plot map if available
            if self.map_data is not None and self.map_info is not None:
                self.plot_map()

            # Plot path if available
            if self.path_data is not None:
                self.plot_path()

            # Set equal aspect ratio
            self.ax.set_aspect('equal')

    def plot_map(self):
        """Plot the occupancy grid map"""
        # Convert occupancy grid to image format
        # 100 = occupied (black), 0 = free (white), -1 = unknown (gray)
        map_img = np.zeros_like(self.map_data, dtype=float)
        map_img[self.map_data == 100] = 0.0  # Occupied = black
        map_img[self.map_data == 0] = 1.0    # Free = white
        map_img[self.map_data == -1] = 0.5   # Unknown = gray

        # Calculate world coordinates
        height, width = self.map_data.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        # World bounds
        x_min = origin_x
        x_max = origin_x + width * resolution
        y_min = origin_y
        y_max = origin_y + height * resolution

        # Display map (flip y-axis because image coordinates are inverted)
        self.ax.imshow(
            np.flipud(map_img),
            extent=[x_min, x_max, y_min, y_max],
            cmap='gray',
            alpha=0.8,
            origin='lower'
        )

    def plot_path(self):
        """Plot the planned path"""
        if len(self.path_data) < 2:
            return

        # Extract x and y coordinates
        x_coords = [point[0] for point in self.path_data]
        y_coords = [point[1] for point in self.path_data]

        # Plot path line
        self.ax.plot(x_coords, y_coords, 'r-', linewidth=3, alpha=0.8, label='Planned Path')

        # Plot start point
        self.ax.plot(x_coords[0], y_coords[0], 'go', markersize=10, label='Start')

        # Plot goal point
        self.ax.plot(x_coords[-1], y_coords[-1], 'ro', markersize=10, label='Goal')

        # Plot waypoints
        self.ax.plot(x_coords[1:-1], y_coords[1:-1], 'bo', markersize=4, alpha=0.6, label='Waypoints')

        # Add legend
        self.ax.legend(loc='upper right')

    def run(self):
        """Run the visualizer"""
        # Start ROS2 spinning in a separate thread
        def spin_thread():
            rclpy.spin(self)

        ros_thread = threading.Thread(target=spin_thread, daemon=True)
        ros_thread.start()

        try:
            # Show the plot
            plt.show()
        except KeyboardInterrupt:
            self.get_logger().info("Visualizer shutting down")
        finally:
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    visualizer = SimpleVisualizer()

    try:
        visualizer.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()