cmake_minimum_required(VERSION 3.8)
project(simulation_bridge)

# --- deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# --- include path ---
include_directories(include)

# --- component as a SHARED library (PLUGIN) ---
add_library(simulation_bridge_lifecycle_component SHARED
  src/simulation_bridge_lifecycle_component.cpp
)
ament_target_dependencies(simulation_bridge_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)

# register the plugin
rclcpp_components_register_nodes(simulation_bridge_lifecycle_component "SimulationBridgeLifecycleNode")

# --- install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  simulation_bridge_lifecycle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch DESTINATION share/${PROJECT_NAME})

ament_package()
