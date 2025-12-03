cmake_minimum_required(VERSION 3.5)
project(slam_tools)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install Python node as an executable
install(
  PROGRAMS
    slam_tools/slam_monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
