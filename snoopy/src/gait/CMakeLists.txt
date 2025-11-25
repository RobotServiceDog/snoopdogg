cmake_minimum_required(VERSION 3.8)
project(gait)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(comm_utils REQUIRED)
find_package(Eigen3 REQUIRED)

# --- include path ---
include_directories(include)

# --- Build Gait libraries ---
add_library(scheduler_lib
    src/scheduler.cpp
)
target_include_directories(scheduler_lib PUBLIC include)
target_compile_options(scheduler_lib PRIVATE -fPIC)

add_library(foot_traj_lib
    src/foot_traj.cpp
)
target_include_directories(foot_traj_lib PUBLIC include)

# --- component as a SHARED library (PLUGIN) ---
add_library(gait_lifecycle_component SHARED
  src/gait_lifecycle_component.cpp
)
ament_target_dependencies(gait_lifecycle_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs comm_utils Eigen3
)

# register the plugin
rclcpp_components_register_nodes(gait_lifecycle_component "GaitLifecycleNode")

target_link_libraries(gait_lifecycle_component
  scheduler_lib foot_traj_lib
)

# --- install ---
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  scheduler_lib
  foot_traj_lib
  gait_lifecycle_component
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
