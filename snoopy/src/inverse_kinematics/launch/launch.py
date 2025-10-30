from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode  # <-- correct import in Humble

def generate_launch_description():
    container = ComposableNodeContainer(
        name='inverse_kinematics_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='inverse_kinematics',
                plugin='InverseKinematicsLifecycleNode',
                name='inverse_kinematics_lifecycle_node',
            ),
        ],
    )

    return LaunchDescription([container])
