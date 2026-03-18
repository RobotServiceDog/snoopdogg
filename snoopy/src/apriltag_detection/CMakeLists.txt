cmake_minimum_required(VERSION 3.8)
project(apriltag_detection)

# --- deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc aruco calib3d)

# --- include path ---
include_directories(include)

# --- component as a SHARED library (PLUGIN) ---
add_library(apriltag_detection_lifecycle_component SHARED
  src/apriltag_detection_lifecycle_component.cpp
)
ament_target_dependencies(apriltag_detection_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components geometry_msgs sensor_msgs cv_bridge
)
target_link_libraries(apriltag_detection_lifecycle_component ${OpenCV_LIBS})

# register the plugin
rclcpp_components_register_nodes(apriltag_detection_lifecycle_component "ApriltagDetectionLifecycleNode")

# --- install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  apriltag_detection_lifecycle_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config DESTINATION share/${PROJECT_NAME})

ament_package()
