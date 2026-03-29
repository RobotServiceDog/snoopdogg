
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    pkg_name = 'servo_control'

    # The container to run your component library
    container = ComposableNodeContainer(
        name='servo_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multithreaded container
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='servo_control::ServoControlLifecycleNode',
                name='servo_control_lifecycle_node',
            ),
        ],
    )

    return LaunchDescription([container])
