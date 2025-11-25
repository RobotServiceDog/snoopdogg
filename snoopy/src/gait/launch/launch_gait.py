from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode  # <-- correct import in Humble

def generate_launch_description():

    pkg_name = 'gait'

    params_file = PathJoinSubstitution([FindPackageShare(pkg_name), 'config', 'gait_params.yaml'])

    container = ComposableNodeContainer(
        name='gait_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='gait',
                plugin='GaitLifecycleNode',
                name='gait_lifecycle_node',
            ),
        ],
    )

    return LaunchDescription([container])