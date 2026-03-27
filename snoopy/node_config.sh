# bash for configuring and activating lifecycle nodes in the snoopy system. This script is intended to be sourced in the terminal after launching the system, and it will set up the necessary environment variables and commands to manage the lifecycle of the nodes.
ros2 lifecycle set /inverse_kinematics_lifecycle_node configure
ros2 lifecycle set /inverse_kinematics_lifecycle_node activate

# timer to delay the configuration and activation of the apriltag_detection node, allowing time for the camera to initialize and start publishing images
sleep 3

ros2 lifecycle set /apriltag_detection_lifecycle_node configure
ros2 lifecycle set /apriltag_detection_lifecycle_node activate