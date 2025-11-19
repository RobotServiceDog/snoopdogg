cmake_minimum_required(VERSION 3.8)
project(servo_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(comm_utils REQUIRED)

# --- pigpiod_if2 (Option 2) ---
find_library(PIGPIO_LIB
  NAMES pigpiod_if2 pigpiod_if pigpio
  PATHS /usr/lib /usr/local/lib
)

if(NOT PIGPIO_LIB)
  message(FATAL_ERROR "pigpio library not found (pigpiod_if2 or pigpio)")
endif()

target_link_libraries(hardware_interface
  ${PIGPIO_LIB}
)


include_directories(include)

# Component library
add_library(servo_control_lifecycle_component SHARED
  src/servo_control_lifecycle_component.cpp
)

ament_target_dependencies(servo_control_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs
)

target_link_libraries(hardware_interface ${PIGPIO_LIBRARIES})

rclcpp_components_register_nodes(servo_control_lifecycle_component "servo_control::ServoControlLifecycleNode")


# Hardware Interface library
add_library(hardware_interface SHARED
  src/hardware_interface.cpp
)

ament_target_dependencies(hardware_interface
  rclcpp rclcpp_lifecycle sensor_msgs comm_utils
)

target_include_directories(hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link hardware interface into component
target_link_libraries(servo_control_lifecycle_component
  hardware_interface
)

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