#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from path_planning.srv import PlanPath
from std_msgs.msg import Header

# Import the AStarPlanner class from the same directory
import sys
import os
sys.path.append(os.path.dirname(__file__))
from astar_planner import AStarPlanner


class PathPlanningService(Node):
    def __init__(self):
        super().__init__('path_planning_service')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('service_name', 'plan_path'),
                ('path_topic', '/path'),
                ('cmd_vel_topic', '/cmd_vel'),
                ('map_topic', '/map'),
                ('default_tolerance', 0.2),
                ('publish_path', True),
                ('max_planning_time', 10.0),  # Maximum time to spend planning
            ])

        # Get parameters
        self.service_name = self.get_parameter('service_name').value
        self.path_topic = self.get_parameter('path_topic').value
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.publish_path = self.get_parameter('publish_path').value
        self.max_planning_time = self.get_parameter('max_planning_time').value

        # Create A* planner instance
        self.planner = AStarPlanner()

        # Service server
        self.path_service = self.create_service(
            PlanPath,
            self.service_name,
            self.plan_path_callback
        )

        # Path publisher (for visualization)
        if self.publish_path:
            self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        # Emergency stop publisher
        self.cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic').value, 10)

        # Map subscriber (delegate to A* planner)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self.map_callback,
            10
        )

        # State variables
        self.current_path = None
        self.planning_active = False

        self.get_logger().info(f"Path Planning Service '{self.service_name}' ready")

    def map_callback(self, msg):
        """Forward map updates to A* planner"""
        self.planner.map_callback(msg)

    def validate_pose(self, pose_stamped, pose_name):
        """Validate that a pose is reasonable"""
        if not pose_stamped.header.frame_id:
            return False, f"{pose_name} pose has no frame_id"

        # Check for NaN or infinite values
        pos = pose_stamped.pose.position
        if (not math.isfinite(pos.x) or not math.isfinite(pos.y) or
            not math.isfinite(pos.z)):
            return False, f"{pose_name} pose has invalid position coordinates"

        # Check orientation (should be normalized quaternion)
        orient = pose_stamped.pose.orientation
        if (not math.isfinite(orient.x) or not math.isfinite(orient.y) or
            not math.isfinite(orient.z) or not math.isfinite(orient.w)):
            return False, f"{pose_name} pose has invalid orientation"

        # Check if quaternion is approximately normalized
        norm = math.sqrt(orient.x**2 + orient.y**2 + orient.z**2 + orient.w**2)
        if abs(norm - 1.0) > 0.1:
            # Try to normalize
            if norm > 0.01:  # Avoid division by zero
                orient.x /= norm
                orient.y /= norm
                orient.z /= norm
                orient.w /= norm
                self.get_logger().warn(f"{pose_name} quaternion was not normalized, fixed")
            else:
                return False, f"{pose_name} pose has zero quaternion"

        return True, "Valid"

    def plan_path_callback(self, request, response):
        """Handle path planning service requests"""
        start_time = time.time()

        try:
            # Validate inputs
            valid_start, start_msg = self.validate_pose(request.start, "Start")
            if not valid_start:
                response.success = False
                response.message = start_msg
                response.planning_time = time.time() - start_time
                return response

            valid_goal, goal_msg = self.validate_pose(request.goal, "Goal")
            if not valid_goal:
                response.success = False
                response.message = goal_msg
                response.planning_time = time.time() - start_time
                return response

            # Check if map is available
            if self.planner.map_data is None:
                response.success = False
                response.message = "No map available for path planning"
                response.planning_time = time.time() - start_time
                return response

            # Convert poses to world coordinates
            start_world = (
                request.start.pose.position.x,
                request.start.pose.position.y
            )
            goal_world = (
                request.goal.pose.position.x,
                request.goal.pose.position.y
            )

            # Log the planning request
            self.get_logger().info(
                f"Planning path from ({start_world[0]:.2f}, {start_world[1]:.2f}) "
                f"to ({goal_world[0]:.2f}, {goal_world[1]:.2f})"
            )

            # Set planning active flag
            self.planning_active = True

            # Plan the path
            path_points = self.planner.plan_path(start_world, goal_world)

            # Check planning time
            planning_time = time.time() - start_time
            if planning_time > self.max_planning_time:
                self.get_logger().warn(f"Path planning took {planning_time:.2f}s (max: {self.max_planning_time}s)")

            if path_points is None or len(path_points) == 0:
                response.success = False
                response.message = "Failed to find a valid path"
                response.planning_time = planning_time
                self.planning_active = False
                return response

            # Create path message
            frame_id = request.start.header.frame_id or "map"
            path_msg = self.planner.create_path_message(path_points, frame_id)

            if path_msg is None:
                response.success = False
                response.message = "Failed to create path message"
                response.planning_time = planning_time
                self.planning_active = False
                return response

            # Store current path
            self.current_path = path_msg

            # Publish path for visualization
            if self.publish_path and self.path_pub:
                self.path_pub.publish(path_msg)

            # Success response
            response.success = True
            response.path = path_msg
            response.message = f"Path planned successfully with {len(path_points)} waypoints"
            response.planning_time = planning_time

            self.get_logger().info(
                f"Path planned successfully: {len(path_points)} points, "
                f"planning time: {planning_time:.3f}s"
            )

        except Exception as e:
            # Handle any unexpected errors
            error_msg = f"Path planning failed with exception: {str(e)}"
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            response.planning_time = time.time() - start_time

        finally:
            self.planning_active = False

        return response

    def emergency_stop(self):
        """Publish emergency stop command"""
        stop_cmd = Twist()  # All zeros
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().warn("Emergency stop command published")

    def get_current_path(self):
        """Get the currently planned path"""
        return self.current_path

    def clear_current_path(self):
        """Clear the current path"""
        self.current_path = None
        self.get_logger().info("Current path cleared")


def main(args=None):
    rclpy.init(args=args)

    service_node = PathPlanningService()

    try:
        rclpy.spin(service_node)
    except KeyboardInterrupt:
        service_node.get_logger().info("Path planning service shutting down")
    except Exception as e:
        service_node.get_logger().error(f"Path planning service error: {e}")
    finally:
        # Emergency stop on shutdown
        try:
            service_node.emergency_stop()
        except:
            pass
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()