import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    package_name='snoopy_description_robot'
    robot_name='snoopy'
    pkg_share_dir = get_package_share_directory(package_name)
    
    
    # --- 3. CORE NODES ---
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share_dir,'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )
    
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[os.path.join(pkg_share_dir, 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_out', '/cmd_vel')] # Map output to a standard name
    )
    
    snoopy_gait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('snoopy_gait_provider'),'launch','launch_gait.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    # follow_controller = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('follow_controller'),'launch','follow_controller.launch.py'
    #     )]), launch_arguments={'use_sim_time': 'false'}.items()
    # )

    # Launch them all!
    return LaunchDescription([
        
        # rsp,
        twist_mux,
        snoopy_gait,
        # follow_controller,
    ])