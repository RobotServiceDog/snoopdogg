cmake_minimum_required(VERSION 3.8)
project(snoopy_description)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_controllers REQUIRED)

# Install config, description, and launch files
install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
