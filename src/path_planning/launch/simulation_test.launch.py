from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('path_planning')

    # Parameters file
    params_file = LaunchConfiguration('params_file')
    map_type = LaunchConfiguration('map_type')

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'path_planning_params.yaml'),
        description='Path to the parameters file')

    declare_map_type_cmd = DeclareLaunchArgument(
        'map_type',
        default_value='room_with_obstacles',
        description='Type of map to simulate: empty, room_with_obstacles, maze, office')

    # Map simulator
    map_simulator_node = Node(
        package='path_planning',
        executable='simple_map_simulator.py',
        name='simple_map_simulator',
        parameters=[{
            'map_type': map_type,
            'map_width': 100,
            'map_height': 100,
            'resolution': 0.1,
            'origin_x': -5.0,
            'origin_y': -5.0,
            'publish_rate': 1.0
        }],
        output='screen'
    )

    # Path planning service (start after map is available)
    path_planning_service_node = TimerAction(
        period=2.0,  # Wait 2 seconds for map to be published
        actions=[
            Node(
                package='path_planning',
                executable='path_planning_service.py',
                name='path_planning_service',
                parameters=[params_file],
                output='screen'
            )
        ]
    )

    # Pure Pursuit controller
    pure_pursuit_node = TimerAction(
        period=3.0,  # Wait 3 seconds for service to be ready
        actions=[
            Node(
                package='path_planning',
                executable='pure_pursuit_controller.py',
                name='pure_pursuit_controller',
                parameters=[params_file],
                output='screen'
            )
        ]
    )

    # Test client (optional, starts after everything else)
    test_client_node = TimerAction(
        period=5.0,  # Wait 5 seconds for everything to be ready
        actions=[
            Node(
                package='path_planning',
                executable='test_path_planning_client.py',
                name='test_path_planning_client',
                parameters=[params_file],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        declare_params_file_cmd,
        declare_map_type_cmd,
        map_simulator_node,
        path_planning_service_node,
        pure_pursuit_node,
        # Uncomment the next line to enable automatic testing
        # test_client_node
    ])