import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    package_name='snoopy_description'
    robot_name='snoopy'
    pkg_share_dir = get_package_share_directory(package_name)
    
    # --- 1. SETUP ENVIRONMENT VARIABLE (FIXES MESH/TF) ---
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', 
        value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', pkg_share_dir]
    )

    # --- 2. ARGUMENTS ---
    default_world_path = os.path.join(
        pkg_share_dir,
        'worlds',
        'empty.sdf'
    )
    
    world_config = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Absolute path to the SDF world file to load'
    )
    
    # --- 3. CORE NODES ---
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share_dir,'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Gazebo Ignition Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
            launch_arguments={'gz_args': '', 'on_exit_shutdown': 'true', 'world': world_config}.items()
    )

    # Spawn Robot Entity (Must complete before controllers start)
    spawn_entity = Node(package='ros_gz_sim', executable='create',
        # FIX: Changed -z from '0.1' to '1.0' to make the dog start floating higher and fall
        arguments=['-topic', 'robot_description', '-name', robot_name, '-z', '5.0'],
        output='screen')

    # --- 4. CONTROLLER NODES ---
    # Spawns the Joint State Broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Main Position Controller
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["snoopy_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --- 5. LAUNCH FLOW CONTROL (THE DISCONNECTION FIX) ---
    
    # Delay controller spawning until the robot entity is successfully created in Gazebo
    delayed_controller_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_broad_spawner, position_controller_spawner],
        )
    )

    # Launch them all!
    return LaunchDescription([
        set_model_path,
        world_arg,
        
        rsp,
        gazebo,
        spawn_entity,
        
        delayed_controller_spawners,
    ])