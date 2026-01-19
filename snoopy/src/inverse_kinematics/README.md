# Inverse Kinematics Module

Subscribes to: `/leg_position_cmd` (comm_utils/msg/LegPosition)

Publishes to: `/snoopy_joint_controller/commands` (std_msgs/msg/Float64MultiArray)

## Usage
To use the module, do the following:
1. Launch the simulation package
```bash
ros2 launch snoopy_description launch_sim.launch.py
```
2. Launch the IK package
```bash
ros2 launch inverse_kinematics inverse_kinematics_launch.py
```
3. Configure/Activate the IK package using the following:
```bash
ros2 topic lifecycle set /inverse_kinematics_lifecycle_node configure
ros2 topic lifecycle set /inverse_kinematics_lifecycle_node activate
```

### Send a position to the IK
```bash
ros2 topic pub -1 /leg_position_cmd comm_utils/msg/LegPosition "$(cat inverse_kinematics/test/position.yaml)"
```