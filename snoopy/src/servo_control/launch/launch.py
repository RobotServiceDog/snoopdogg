
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    pkg_name = 'servo_control'

    # Hardcode parameter YAML path
    params_file = PathJoinSubstitution([
        FindPackageShare(pkg_name),
        'config',
        'servo_control_parameters.yaml'
    ])

    # Composable node
    composable_node = ComposableNode(
        package=pkg_name,
        plugin='ServoControlLifecycleNode',
        name='servo_control_node',
        parameters=[params_file],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # Container
    container = ComposableNodeContainer(
        name='servo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[composable_node],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(container)
    return ld
