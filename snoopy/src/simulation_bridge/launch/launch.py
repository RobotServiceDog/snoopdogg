from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='simulation_bridge_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='simulation_bridge',
                plugin='SimulationBridgeLifecycleNode',
                name='simulation_bridge_lifecycle_node',
            ),
        ],
    )

    return LaunchDescription([container])
