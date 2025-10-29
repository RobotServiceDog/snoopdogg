cmake_minimum_required(VERSION 3.8)
project(servo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# find pigpio daemon client library
find_library(PIGPIOD_IF2_LIB NAMES pigpiod_if2 REQUIRED)

# Executable (standalone lifecycle node)
add_executable(servo_lifecycle_component src/servo_lifecycle_component.cpp)

ament_target_dependencies(servo_lifecycle_component
  rclcpp 
  rclcpp_lifecycle 
  lifecycle_msgs)

target_link_libraries(servo_lifecycle_component
  ${PIGPIOD_IF2_LIB}
  pthread
  rt)

install(TARGETS servo_lifecycle_component DESTINATION lib/${PROJECT_NAME})

# Component (loadable)
add_library(servo_lifecycle_component SHARED src/servo_lifecycle_component.cpp)

ament_target_dependencies(servo_lifecycle_component
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs)

target_link_libraries(servo_lifecycle_component 
  ${PIGPIOD_IF2_LIB} 
  pthread
  rt)

# Register component
find_package(rclcpp_components REQUIRED)

rclcpp_components_register_nodes(servo_lifecycle_component "ServoLifecycleNode")

install(TARGETS servo_lifecycle_component DESTINATION lib)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
