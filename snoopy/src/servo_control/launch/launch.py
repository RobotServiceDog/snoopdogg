from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode  # <-- correct import in Humble

def generate_launch_description():
    container = ComposableNodeContainer(
        name='servo_control_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='servo_control',
                plugin='ServoControlLifecycleNode',
                name='servo_control_lifecycle_node',
            ),
        ],
    )

    return LaunchDescription([container])
