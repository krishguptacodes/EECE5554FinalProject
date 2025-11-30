#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped
from path_planning.srv import PlanPath


class TestPathPlanningClient(Node):
    def __init__(self):
        super().__init__('test_path_planning_client')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('service_name', 'plan_path'),
                ('frame_id', 'map'),
                ('test_interval', 10.0),  # seconds between test requests
            ])

        self.service_name = self.get_parameter('service_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.test_interval = self.get_parameter('test_interval').value

        # Service client
        self.path_client = self.create_client(PlanPath, self.service_name)

        # Timer for periodic testing
        self.test_timer = self.create_timer(self.test_interval, self.run_test)

        self.test_counter = 0

        self.get_logger().info(f"Test Path Planning Client initialized, testing service: {self.service_name}")

    def create_pose(self, x, y, yaw=0.0):
        """Create a PoseStamped message"""
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def run_test(self):
        """Run a path planning test"""
        if not self.path_client.service_is_ready():
            self.get_logger().warn(f"Service {self.service_name} not ready")
            return

        # Define test scenarios
        test_scenarios = [
            # (start_x, start_y, goal_x, goal_y, description)
            (0.0, 0.0, 2.0, 2.0, "diagonal movement"),
            (-1.0, -1.0, 1.0, 1.0, "cross center"),
            (0.0, 0.0, 3.0, 0.0, "straight line"),
            (1.0, 1.0, -1.0, -1.0, "return to origin"),
        ]

        scenario_idx = self.test_counter % len(test_scenarios)
        start_x, start_y, goal_x, goal_y, description = test_scenarios[scenario_idx]

        # Create request
        request = PlanPath.Request()
        request.start = self.create_pose(start_x, start_y)
        request.goal = self.create_pose(goal_x, goal_y)
        request.tolerance = 0.2

        self.get_logger().info(
            f"Test {self.test_counter + 1}: {description} - "
            f"from ({start_x}, {start_y}) to ({goal_x}, {goal_y})"
        )

        # Send async request
        future = self.path_client.call_async(request)
        future.add_done_callback(self.handle_response)

        self.test_counter += 1

    def handle_response(self, future):
        """Handle service response"""
        try:
            response = future.result()

            if response.success:
                path_length = len(response.path.poses)
                self.get_logger().info(
                    f"✓ Path planning successful: {path_length} waypoints, "
                    f"planning time: {response.planning_time:.3f}s"
                )
                self.get_logger().info(f"  Message: {response.message}")

                # Log first and last waypoints
                if path_length > 0:
                    first = response.path.poses[0].pose.position
                    last = response.path.poses[-1].pose.position
                    self.get_logger().info(
                        f"  Path: ({first.x:.2f}, {first.y:.2f}) → "
                        f"({last.x:.2f}, {last.y:.2f})"
                    )
            else:
                self.get_logger().error(
                    f"✗ Path planning failed: {response.message}, "
                    f"planning time: {response.planning_time:.3f}s"
                )

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def test_single_path(self, start_x, start_y, goal_x, goal_y):
        """Test a single path planning request (for manual testing)"""
        if not self.path_client.service_is_ready():
            self.get_logger().error(f"Service {self.service_name} not ready")
            return False

        request = PlanPath.Request()
        request.start = self.create_pose(start_x, start_y)
        request.goal = self.create_pose(goal_x, goal_y)
        request.tolerance = 0.2

        try:
            future = self.path_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

            if future.done():
                response = future.result()
                return response.success
            else:
                self.get_logger().error("Service call timed out")
                return False

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)

    client = TestPathPlanningClient()

    try:
        # Wait for service to be available
        client.get_logger().info("Waiting for path planning service...")
        if client.path_client.wait_for_service(timeout_sec=30.0):
            client.get_logger().info("Path planning service available, starting tests")
            rclpy.spin(client)
        else:
            client.get_logger().error("Path planning service not available")

    except KeyboardInterrupt:
        client.get_logger().info("Test client shutting down")
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()