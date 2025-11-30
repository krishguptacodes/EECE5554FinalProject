#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import heapq
import numpy as np

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header


class Node2D:
    """A node in the A* search grid"""
    def __init__(self, x, y, cost=0.0, heuristic=0.0, parent=None):
        self.x = x
        self.y = y
        self.cost = cost  # g(n) - actual cost from start
        self.heuristic = heuristic  # h(n) - heuristic cost to goal
        self.parent = parent

    @property
    def f_cost(self):
        """Total cost f(n) = g(n) + h(n)"""
        return self.cost + self.heuristic

    def __lt__(self, other):
        return self.f_cost < other.f_cost


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_topic', '/map'),
                ('allow_diagonal', True),
                ('heuristic_weight', 1.0),
                ('inflation_radius', 0.2),  # meters
                ('robot_radius', 0.2),      # meters
            ])

        # Get parameters
        self.allow_diagonal = self.get_parameter('allow_diagonal').value
        self.heuristic_weight = self.get_parameter('heuristic_weight').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.robot_radius = self.get_parameter('robot_radius').value

        # Map subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            10
        )

        # State variables
        self.map_data = None
        self.map_info = None
        self.inflated_map = None

        self.get_logger().info("A* Planner initialized")

    def map_callback(self, msg):
        """Receive and process occupancy grid map"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info

        # Inflate obstacles for robot safety
        self.inflated_map = self.inflate_obstacles(self.map_data)

        self.get_logger().info(f"Received map: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}m/pixel")

    def inflate_obstacles(self, grid_map):
        """Inflate obstacles by robot radius for safe path planning"""
        if self.map_info is None:
            return grid_map

        # Calculate inflation radius in grid cells
        inflation_cells = int(self.inflation_radius / self.map_info.resolution)

        inflated = grid_map.copy()
        height, width = grid_map.shape

        # Find all obstacle cells
        obstacle_cells = np.where(grid_map > 50)  # Occupied threshold

        # Inflate around each obstacle
        for oy, ox in zip(obstacle_cells[0], obstacle_cells[1]):
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                        ny, nx = oy + dy, ox + dx
                        if 0 <= ny < height and 0 <= nx < width:
                            inflated[ny, nx] = 100  # Mark as occupied

        return inflated

    def world_to_grid(self, world_x, world_y):
        """Convert world coordinates to grid coordinates"""
        if self.map_info is None:
            return None, None

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        grid_x = int((world_x - origin_x) / resolution)
        grid_y = int((world_y - origin_y) / resolution)

        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates"""
        if self.map_info is None:
            return None, None

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution

        world_x = origin_x + grid_x * resolution
        world_y = origin_y + grid_y * resolution

        return world_x, world_y

    def is_valid_cell(self, x, y):
        """Check if grid cell is valid and not occupied"""
        if self.inflated_map is None:
            return False

        height, width = self.inflated_map.shape

        # Check bounds
        if x < 0 or x >= width or y < 0 or y >= height:
            return False

        # Check if cell is free (value < 50 means free, > 50 means occupied)
        return self.inflated_map[y, x] < 50

    def heuristic_distance(self, x1, y1, x2, y2):
        """Calculate heuristic distance between two grid points"""
        if self.allow_diagonal:
            # Diagonal distance (Chebyshev distance)
            dx = abs(x2 - x1)
            dy = abs(y2 - y1)
            return max(dx, dy) + (math.sqrt(2) - 1) * min(dx, dy)
        else:
            # Manhattan distance
            return abs(x2 - x1) + abs(y2 - y1)

    def get_neighbors(self, x, y):
        """Get valid neighboring cells"""
        neighbors = []

        # 8-connected or 4-connected movement
        if self.allow_diagonal:
            directions = [(-1, -1), (-1, 0), (-1, 1),
                         (0, -1),           (0, 1),
                         (1, -1),  (1, 0),  (1, 1)]
        else:
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if self.is_valid_cell(nx, ny):
                # Calculate movement cost
                if abs(dx) + abs(dy) == 2:  # Diagonal movement
                    cost = math.sqrt(2)
                else:  # Straight movement
                    cost = 1.0
                neighbors.append((nx, ny, cost))

        return neighbors

    def reconstruct_path(self, goal_node):
        """Reconstruct path from goal to start"""
        path_points = []
        current = goal_node

        while current is not None:
            world_x, world_y = self.grid_to_world(current.x, current.y)
            if world_x is not None and world_y is not None:
                path_points.append((world_x, world_y))
            current = current.parent

        # Reverse to get start-to-goal path
        path_points.reverse()
        return path_points

    def smooth_path(self, path_points):
        """Apply simple path smoothing to reduce jagged edges"""
        if len(path_points) < 3:
            return path_points

        smoothed = [path_points[0]]  # Always keep start point

        i = 0
        while i < len(path_points) - 1:
            # Look ahead to find the furthest point we can reach directly
            furthest = i + 1

            # Be more conservative with smoothing - limit lookahead distance
            max_lookahead = min(i + 5, len(path_points))  # Limit to 5 points ahead

            for j in range(i + 2, max_lookahead):
                # Calculate distance to avoid overly long jumps
                start_point = path_points[i]
                test_point = path_points[j]
                distance = ((test_point[0] - start_point[0])**2 + (test_point[1] - start_point[1])**2)**0.5

                # Limit maximum jump distance (in meters) - be more permissive
                max_jump_distance = 3.0  # increased from 2.0 to 3.0 meters

                if distance > max_jump_distance:
                    self.get_logger().debug(f"Skipping smoothing: distance {distance:.2f}m > {max_jump_distance}m")
                    break

                if self.line_of_sight(path_points[i], path_points[j]):
                    furthest = j
                else:
                    self.get_logger().debug(f"Line of sight blocked from point {i} to {j}")
                    break

            smoothed.append(path_points[furthest])
            i = furthest

        self.get_logger().debug(f"Smoothing: {len(path_points)} -> {len(smoothed)} points")
        return smoothed

    def line_of_sight(self, start_point, end_point):
        """Check if there's a clear line of sight between two world points"""
        # Convert to grid coordinates
        x0, y0 = self.world_to_grid(start_point[0], start_point[1])
        x1, y1 = self.world_to_grid(end_point[0], end_point[1])

        if x0 is None or y0 is None or x1 is None or y1 is None:
            self.get_logger().debug(f"Invalid coordinates in line_of_sight: {start_point} -> {end_point}")
            return False

        # Add safety margin - check if endpoints are valid
        if not self.is_valid_cell(x0, y0) or not self.is_valid_cell(x1, y1):
            self.get_logger().debug(f"Start or end point not valid: ({x0},{y0}) or ({x1},{y1})")
            return False

        # For very short distances, still check the path
        distance = abs(x1 - x0) + abs(y1 - y0)
        if distance < 2:
            # Even for short distances, check intermediate points
            return self.check_line_with_safety_margin(x0, y0, x1, y1)

        # Use denser line checking algorithm
        return self.check_line_with_safety_margin(x0, y0, x1, y1)

    def check_line_with_safety_margin(self, x0, y0, x1, y1):
        """Check line of sight with safety margin for thin obstacles"""
        # Use supersampling - check more points along the line
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        # Calculate the number of steps - use more steps for denser checking
        steps = max(dx, dy) * 2  # Double density
        if steps == 0:
            return True

        # Check points along the line using linear interpolation
        for i in range(steps + 1):
            t = i / steps if steps > 0 else 0
            x = int(round(x0 + t * (x1 - x0)))
            y = int(round(y0 + t * (y1 - y0)))

            # Check the main point
            if not self.is_valid_cell(x, y):
                self.get_logger().debug(f"Obstacle detected at ({x},{y}) during line check")
                return False

            # Add safety margin by checking adjacent cells for thin obstacles
            # This helps detect narrow passages that might be missed
            safety_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            obstacle_count = 0

            for dx_offset, dy_offset in safety_offsets:
                check_x = x + dx_offset
                check_y = y + dy_offset
                if not self.is_valid_cell(check_x, check_y):
                    obstacle_count += 1

            # If surrounded by too many obstacles, it's likely a narrow passage
            if obstacle_count >= 3:
                self.get_logger().debug(f"Narrow passage detected at ({x},{y}) - obstacle count: {obstacle_count}")
                return False

        return True

    def plan_path(self, start_world, goal_world):
        """Plan path from start to goal using A* algorithm"""
        if self.inflated_map is None:
            self.get_logger().error("No map available for path planning")
            return None

        # Convert world coordinates to grid
        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.world_to_grid(goal_world[0], goal_world[1])

        if start_x is None or goal_x is None:
            self.get_logger().error("Invalid start or goal coordinates")
            return None

        # Check if start and goal are valid
        if not self.is_valid_cell(start_x, start_y):
            self.get_logger().error(f"Start position ({start_x}, {start_y}) is not valid")
            return None

        if not self.is_valid_cell(goal_x, goal_y):
            self.get_logger().error(f"Goal position ({goal_x}, {goal_y}) is not valid")
            return None

        # A* algorithm
        open_list = []
        closed_set = set()

        # Create start node
        start_node = Node2D(
            start_x, start_y,
            cost=0.0,
            heuristic=self.heuristic_weight * self.heuristic_distance(start_x, start_y, goal_x, goal_y)
        )

        heapq.heappush(open_list, start_node)

        # For tracking visited nodes efficiently
        node_map = {(start_x, start_y): start_node}

        iterations = 0
        max_iterations = 10000

        while open_list and iterations < max_iterations:
            iterations += 1

            # Get node with lowest f_cost
            current_node = heapq.heappop(open_list)
            current_pos = (current_node.x, current_node.y)

            # Check if we reached the goal
            if current_node.x == goal_x and current_node.y == goal_y:
                self.get_logger().info(f"Path found in {iterations} iterations")
                path_points = self.reconstruct_path(current_node)

                # Apply path smoothing with safer parameters
                smoothed_path = self.smooth_path(path_points)
                self.get_logger().info(f"Path smoothed from {len(path_points)} to {len(smoothed_path)} points")

                # Debug: log original and smoothed path
                self.get_logger().info(f"Original path: {path_points[:3]}...{path_points[-3:] if len(path_points) > 3 else path_points}")
                self.get_logger().info(f"Smoothed path: {smoothed_path}")

                return smoothed_path

            closed_set.add(current_pos)

            # Explore neighbors
            for neighbor_x, neighbor_y, move_cost in self.get_neighbors(current_node.x, current_node.y):
                neighbor_pos = (neighbor_x, neighbor_y)

                if neighbor_pos in closed_set:
                    continue

                # Calculate costs
                tentative_cost = current_node.cost + move_cost
                heuristic = self.heuristic_weight * self.heuristic_distance(neighbor_x, neighbor_y, goal_x, goal_y)

                # Check if this is a better path to the neighbor
                if neighbor_pos in node_map:
                    neighbor_node = node_map[neighbor_pos]
                    if tentative_cost < neighbor_node.cost:
                        neighbor_node.cost = tentative_cost
                        neighbor_node.parent = current_node
                        heapq.heappush(open_list, neighbor_node)
                else:
                    # New node
                    neighbor_node = Node2D(
                        neighbor_x, neighbor_y,
                        cost=tentative_cost,
                        heuristic=heuristic,
                        parent=current_node
                    )
                    node_map[neighbor_pos] = neighbor_node
                    heapq.heappush(open_list, neighbor_node)

        self.get_logger().error("Path planning failed - no path found")
        return None

    def create_path_message(self, path_points, frame_id="map"):
        """Create ROS Path message from list of world coordinates"""
        if not path_points:
            return None

        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i, (x, y) in enumerate(path_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Calculate orientation based on direction to next point
            if i < len(path_points) - 1:
                next_x, next_y = path_points[i + 1]
                yaw = math.atan2(next_y - y, next_x - x)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
            else:
                # Last point - use previous orientation
                pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        return path_msg


def main(args=None):
    rclpy.init(args=args)

    planner = AStarPlanner()

    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()