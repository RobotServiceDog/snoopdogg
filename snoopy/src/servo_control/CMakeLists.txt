cmake_minimum_required(VERSION 3.8)
project(servo_control)

# --- deps ---
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)

find_package(generate_parameter_library REQUIRED)

# --- include path ---
include_directories(include)

# --- parameter library generation ---
generate_parameter_library(servo_control_params config/servo_control_parameters.yaml)

# --- component as a SHARED library (PLUGIN) ---
add_library(servo_control_lifecycle_component SHARED
  src/servo_control_lifecycle_component.cpp
)

ament_target_dependencies(servo_control_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)

# Link your parameter library here ⬇️
target_link_libraries(servo_control_lifecycle_component
  servo_control_params
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

install(DIRECTORY
  launch DESTINATION share/${PROJECT_NAME})

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)


ament_package()
