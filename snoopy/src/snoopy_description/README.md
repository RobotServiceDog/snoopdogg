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
