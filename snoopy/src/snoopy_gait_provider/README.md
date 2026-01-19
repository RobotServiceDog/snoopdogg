# Snoopy Gait Provider

This package provides a gait scheduler for a quadruped robot. Specifically, it uses the trot gait to generate leg position commands.

Subscribes to: None

Publishes to: `/leg_position_cmd` (comm_utils/msg/LegPosition)

## Usage
To use the gait provider, follow these steps:
1. Launch the simulation package:
```bash
ros2 launch snoopy_description launch_sim.launch.py
```
2. Launch the IK package:
```bash
ros2 launch inverse_kinematics inverse_kinematics_launch.py
```
3. Configure/Activate the IK package using the following:
```bash
ros2 topic lifecycle set /inverse_kinematics_lifecycle_node configure
ros2 topic lifecycle set /inverse_kinematics_lifecycle_node activate
```
4. Launch the gait provider:
```bash
ros2 launch snoopy_gait_provider trot_gait_launch.py
```

## Gait Details
The trot gait alternates diagonal pairs of legs. The front-left and back-right legs move together, while the front-right and back-left legs move together. The gait cycle is defined by parameters such as step height, step length, and cycle duration.

**Parameters:**
- Step Frequency: 2.5 Hz
    - How many cycles per second
- Step Length: 0.03 meters
    - This, combined with step frequency, determines forward speed
- Step Height: 0.03 meters
- Base Height: 0.15 meters
    - How tall the robot is will trotting