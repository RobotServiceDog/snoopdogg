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
        'basic.sdf'
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
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_config],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot Entity (higher so we can see it fall and land on feet)
    spawn_entity = Node(package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', robot_name, '-z', '0.25'],
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
    
    # --- 6. THE BRIDGE (CLOCK AND TOPICS) ---
    # This is required for Ignition to send the /clock to ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/snoopy/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/snoopy/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[os.path.join(pkg_share_dir, 'config', 'twist_mux.yaml')],
        remappings=[('/cmd_vel_joy', '/cmd_vel')] # Map output to a standard name
    )
    
    snoopy_gait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('snoopy_gait_provider'),'launch','launch_gait.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    follow_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('follow_controller'),'launch','follow_controller.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    inverse_kinematics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('inverse_kinematics'),'launch','launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    apriltag_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('apriltag_detection'),'launch','launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch them all!
    return LaunchDescription([
        set_model_path,
        world_arg,
        
        rsp,
        gazebo,
        spawn_entity,
        bridge,

        twist_mux,
        apriltag_detection,
        follow_controller,
        snoopy_gait,
        inverse_kinematics,
        
        delayed_controller_spawners,
    ])