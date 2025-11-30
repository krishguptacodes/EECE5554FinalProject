cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Navigation stack dependencies (commented for basic testing)
# find_package(nav2_msgs REQUIRED)
# find_package(nav2_common REQUIRED)
# find_package(nav2_util REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanPath.srv"
  DEPENDENCIES geometry_msgs nav_msgs
)

# Include directories
include_directories(include)

# Install Python executables
install(PROGRAMS
  scripts/pure_pursuit_controller.py
  scripts/test_tf_publisher.py
  scripts/astar_planner.py
  scripts/path_planning_service.py
  scripts/test_path_planning_client.py
  scripts/simple_map_simulator.py
  scripts/simple_visualizer.py
  scripts/interactive_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies for other packages
ament_export_dependencies(rosidl_default_runtime)

ament_package()