cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

# --- deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# --- include path ---
include_directories(include)

# --- component as a SHARED library (PLUGIN) ---
add_library(inverse_kinematics_lifecycle_component SHARED
  src/inverse_kinematics_lifecycle_component.cpp
)
ament_target_dependencies(inverse_kinematics_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)

# register the plugin
rclcpp_components_register_nodes(inverse_kinematics_lifecycle_component "InverseKinematicsLifecycleNode")

# --- install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  inverse_kinematics_lifecycle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch DESTINATION share/${PROJECT_NAME})

ament_package()
