cmake_minimum_required(VERSION 3.8)
project(follow_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Define the executable
add_executable(follow_node 
  src/follow_node.cpp
)

# Include headers
target_include_directories(follow_node PUBLIC include)

# Link dependencies
ament_target_dependencies(follow_node
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install include files
install(DIRECTORY include/
  DESTINATION include
)

# Install executable
install(TARGETS
  follow_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Launch and Config folders
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()