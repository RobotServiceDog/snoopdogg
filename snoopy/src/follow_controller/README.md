# Follow Controller Module

Subscribes to: `/odom` (nav_msgs/msg/Odometry), `/target_pose` (geometry_msgs/msg/PoseStamped)

Publishes to: `/cmd_vel` (geometry_msgs/msg/Twist)

## Usage
To use the module, do the following:
1. Launch the simulation package
```bash
ros2 launch snoopy_description launch_sim.launch.py
```

This will automatically include and launch the follow controller node along with other necessary components.


If you want to just launch the follow controller separately, you can do so by running:
```bash
ros2 launch follow_controller follow_controller.launch.py
```

### Controller Parameters
The follow controller uses the following parameters, found in `/config/params.yaml`:
```yaml
follow_controller:
  ros__parameters:
    target_topic: "/goal_pose"
    odom_topic: "/odom"
    cmd_topic: "/cmd_vel"
    control_frequency: 50.0
    kp_linear: 0.8
    kd_linear: 0.1
    kp_angular: 2.0
    kd_angular: 0.05
    goal_tolerance: 0.2
```