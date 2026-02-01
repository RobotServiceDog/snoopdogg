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

### Give a Target Pose
To set a target pose for the robot to follow, publish a `geometry_msgs/msg/PoseStamped` message to the `/goal_pose` topic. **Note, currently the goal pose must be in the `world` frame.**

You can also use Rviz2 to set a 2D Nav Goal, which will publish to the `/goal_pose` topic.
```bash
rviz2
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