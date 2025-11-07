import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('snoopy_description')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Xacro command to generate URDF content
    robot_description_path = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_content = ParameterValue(
        Command([
            'xacro ', robot_description_path,
            ' use_sim_time:=', use_sim_time,
            ' use_ros2_control:=', use_ros2_control,
        ]),
        value_type=str
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content, 
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control to manage joints'),
        
        robot_state_publisher_node
    ])