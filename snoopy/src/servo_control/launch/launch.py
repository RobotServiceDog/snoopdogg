
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_name = 'servo_control'

    # Path to your parameters YAML
    params_file = PathJoinSubstitution([FindPackageShare(pkg_name), 'config', 'servo_control_parameters.yaml'])

    # The container to run your component library
    container = ComposableNodeContainer(
        name='servo_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded container
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ServoControlLifecycleNode',
                name='servo_control_node',
                namespace='',
                parameters=[params_file],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
