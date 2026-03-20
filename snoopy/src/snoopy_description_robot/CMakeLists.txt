cmake_minimum_required(VERSION 3.8)
project(snoopy_description_robot)

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install config, description, and launch files
install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
