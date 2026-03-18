from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode  # <-- correct import in Humble

def generate_launch_description():

    pkg_name = 'apriltag_detection'
    params_file = PathJoinSubstitution([FindPackageShare(pkg_name), 'config', 'apriltag_params.yaml'])

    container = ComposableNodeContainer(
        name='apriltag_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_detection',
                plugin='ApriltagDetectionLifecycleNode',
                name='apriltag_detection_lifecycle_node',
                parameters=[params_file],
            ),
        ],
    )

    return LaunchDescription([container])
