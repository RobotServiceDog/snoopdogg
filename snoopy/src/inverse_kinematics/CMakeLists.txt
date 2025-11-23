cmake_minimum_required(VERSION 3.8)
project(inverse_kinematics)

# --- deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(comm_utils REQUIRED)

# --- include path ---
include_directories(include)

# --- Build IK library (pure C++ math) ---
add_library(inverse_kinematics_lib
    src/inverse_kinematics.cpp
)
target_include_directories(inverse_kinematics_lib PUBLIC include)

# --- component as a SHARED library (PLUGIN) ---
add_library(inverse_kinematics_lifecycle_component SHARED
  src/inverse_kinematics_lifecycle_component.cpp
)
ament_target_dependencies(inverse_kinematics_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs sensor_msgs comm_utils
)

# register the plugin
rclcpp_components_register_nodes(inverse_kinematics_lifecycle_component "InverseKinematicsLifecycleNode")

target_link_libraries(inverse_kinematics_lifecycle_component
  inverse_kinematics_lib
)

# --- install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  inverse_kinematics_lib
  inverse_kinematics_lifecycle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch DESTINATION share/${PROJECT_NAME})

# --- install config files ---
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

ament_package()