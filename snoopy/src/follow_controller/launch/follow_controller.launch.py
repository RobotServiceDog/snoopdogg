import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    # Path to the yaml file
    config = os.path.join(
        get_package_share_directory('follow_controller'),
        'config',
        'params.yaml'
    )

    follow_controller_node = Node(
            package='follow_controller',
            executable='follow_node',
            name='follow_controller',
            parameters=[config],
            output='screen'
        )
    
    
    return LaunchDescription([
        use_sim_time_arg,
        follow_controller_node
    ])