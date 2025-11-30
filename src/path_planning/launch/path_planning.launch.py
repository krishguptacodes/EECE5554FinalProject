from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_dir = get_package_share_directory('path_planning')

    # Parameters file
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'path_planning_params.yaml'),
        description='Path to the parameters file')

    # Pure Pursuit controller node
    pure_pursuit_node = Node(
        package='path_planning',
        executable='pure_pursuit_controller.py',
        name='pure_pursuit_controller',
        parameters=[params_file],
        output='screen'
    )

    # TF publisher for testing
    tf_publisher_node = Node(
        package='path_planning',
        executable='test_tf_publisher.py',
        name='test_tf_publisher',
        output='screen'
    )

    # Path planning service
    path_planning_service_node = Node(
        package='path_planning',
        executable='path_planning_service.py',
        name='path_planning_service',
        parameters=[params_file],
        output='screen'
    )

    # Test client (commented out by default)
    # test_client_node = Node(
    #     package='path_planning',
    #     executable='test_path_planning_client.py',
    #     name='test_path_planning_client',
    #     output='screen'
    # )

    return LaunchDescription([
        declare_params_file_cmd,
        pure_pursuit_node,
        tf_publisher_node,
        path_planning_service_node
    ])