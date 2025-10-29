cmake_minimum_required(VERSION 3.8)
project(servo_control)

# --- deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# --- include path ---
include_directories(include)

# --- component as a SHARED library (PLUGIN) ---
add_library(servo_control_lifecycle_component SHARED
  src/servo_control_lifecycle_component.cpp
)
ament_target_dependencies(servo_control_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)

# register the plugin
rclcpp_components_register_nodes(servo_control_lifecycle_component "ServoControlLifecycleNode")

# --- install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  servo_control_lifecycle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
