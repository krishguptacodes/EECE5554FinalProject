#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys

from path_planning.srv import PlanPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class InteractivePlanner(Node):
    def __init__(self):
        super().__init__('interactive_planner')

        # Service client
        self.path_client = self.create_client(PlanPath, 'plan_path')

        self.get_logger().info("Interactive Path Planner started!")
        self.get_logger().info("Waiting for path planning service...")

    def plan_path_interactive(self):
        """Interactive path planning with user input"""

        # Wait for service
        if not self.path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Path planning service not available!")
            return

        print("\n" + "="*60)
        print("🗺️  INTERACTIVE PATH PLANNER")
        print("="*60)
        print("Map range: -5.0 to 5.0 meters (X and Y)")
        print("Safe areas are usually away from (0,0) center")
        print("Try coordinates like: -3.5, -2.0, 2.0, 3.5")
        print("="*60)

        while True:
            try:
                print("\n📍 SET START POINT:")
                start_x = float(input("Start X coordinate (meters): "))
                start_y = float(input("Start Y coordinate (meters): "))

                print("\n🎯 SET GOAL POINT:")
                goal_x = float(input("Goal X coordinate (meters): "))
                goal_y = float(input("Goal Y coordinate (meters): "))

                tolerance = float(input("Tolerance (0.1-0.5 meters, default 0.2): ") or "0.2")

                print(f"\n🚀 Planning path from ({start_x:.1f}, {start_y:.1f}) to ({goal_x:.1f}, {goal_y:.1f})")
                print("Planning...")

                # Create service request
                request = PlanPath.Request()

                # Start pose
                request.start = PoseStamped()
                request.start.header.frame_id = 'map'
                request.start.header.stamp = self.get_clock().now().to_msg()
                request.start.pose.position.x = start_x
                request.start.pose.position.y = start_y
                request.start.pose.position.z = 0.0
                request.start.pose.orientation.w = 1.0

                # Goal pose
                request.goal = PoseStamped()
                request.goal.header.frame_id = 'map'
                request.goal.header.stamp = self.get_clock().now().to_msg()
                request.goal.pose.position.x = goal_x
                request.goal.pose.position.y = goal_y
                request.goal.pose.position.z = 0.0
                request.goal.pose.orientation.w = 1.0

                request.tolerance = tolerance

                # Call service
                future = self.path_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

                if future.done():
                    response = future.result()
                    self.print_result(response, start_x, start_y, goal_x, goal_y)
                else:
                    print("❌ Service call timed out!")

            except ValueError:
                print("❌ Invalid input! Please enter numbers only.")
                continue
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                continue

            # Ask if user wants to continue
            print("\n" + "-"*40)
            choice = input("Plan another path? (y/n): ").lower()
            if choice not in ['y', 'yes']:
                print("👋 Goodbye!")
                break

    def print_result(self, response, start_x, start_y, goal_x, goal_y):
        """Print formatted result"""
        print("\n" + "="*50)

        if response.success:
            print("✅ PATH PLANNING SUCCESSFUL!")
            print(f"📊 Waypoints: {len(response.path.poses)}")
            print(f"⏱️  Planning time: {response.planning_time:.3f} seconds")
            print(f"💬 Message: {response.message}")

            if len(response.path.poses) > 0:
                print("\n🛣️  PATH WAYPOINTS:")
                for i, pose in enumerate(response.path.poses):
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    if i == 0:
                        print(f"   🚩 Start: ({x:.2f}, {y:.2f})")
                    elif i == len(response.path.poses) - 1:
                        print(f"   🎯 Goal:  ({x:.2f}, {y:.2f})")
                    else:
                        print(f"   📍 Point {i}: ({x:.2f}, {y:.2f})")

                # Calculate total distance
                total_distance = 0.0
                for i in range(1, len(response.path.poses)):
                    p1 = response.path.poses[i-1].pose.position
                    p2 = response.path.poses[i].pose.position
                    dist = ((p2.x - p1.x)**2 + (p2.y - p1.y)**2)**0.5
                    total_distance += dist

                print(f"\n📏 Total path distance: {total_distance:.2f} meters")

        else:
            print("❌ PATH PLANNING FAILED!")
            print(f"💬 Reason: {response.message}")
            print(f"⏱️  Time: {response.planning_time:.3f} seconds")

            print("\n💡 TROUBLESHOOTING TIPS:")
            print("   • Try coordinates further from map center (0,0)")
            print("   • Avoid areas with dense obstacles")
            print("   • Use coordinates between -4.0 and 4.0")
            print("   • Increase tolerance to 0.3 or 0.5")

        print("="*50)


def main(args=None):
    rclpy.init(args=args)

    planner = InteractivePlanner()

    try:
        planner.plan_path_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()