# snoopy_description
This package contains the URDF description and Gazebo simulation launch files for the Snoopy robot.

## Contents
- `description/`: Contains the URDF and Xacro files defining the Snoopy robot model.
- `launch/`: Contains launch files to start the Gazebo simulation with the Snoopy robot

## Launching the Simulation
To launch the Gazebo simulation with the Snoopy robot, use the following command:
```bash
ros2 launch snoopy_description launch_sim.launch.py
```
Make sure to run this from the `src/` directory of your ROS2 workspace.

## Robot Model
The robot model is defined in the `description/robot.urdf.xacro` file, which includes the leg definitions and ROS2 control configurations.



# Running Commands


```bash
ros2 launch snoopy_description_robot launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard

OR

ros2 launch apriltag_detection launch.py
ros2 launch follow_controller follow_controller.launch.py
```

```bash
ros2 launch snoopy_gait_provider launch_gait.launch.py
```

```bash
ros2 launch inverse_kinematics launch.py
```

```bash
ros2 launch servo_control launch.py
```

```bash
ros2 lifecycle get
ros2 lifecycle set /apriltag_detection_lifecycle_node configure
ros2 lifecycle set /apriltag_detection_lifecycle_node activate
ros2 lifecycle set /inverse_kinematics_lifecycle_node configure
ros2 lifecycle set /inverse_kinematics_lifecycle_node activate
ros2 lifecycle set /servo_control_lifecycle_node configure
ros2 lifecycle set /servo_control_lifecycle_node activate
ros2 lifecycle get
````

