cmake_minimum_required(VERSION 3.8)
project(servo_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(comm_utils REQUIRED)

include_directories(include)

# Component library
add_library(servo_control_lifecycle_component SHARED
  src/servo_control_lifecycle_component.cpp
)

ament_target_dependencies(servo_control_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)

rclcpp_components_register_nodes(servo_control_lifecycle_component "servo_control::ServoControlLifecycleNode")


# --- install headers ---
install(DIRECTORY include/
        DESTINATION include)

# --- install library ---
install(TARGETS servo_control_lifecycle_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

# --- install launch files ---
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

# --- install config files ---
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()