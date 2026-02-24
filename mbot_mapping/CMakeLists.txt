cmake_minimum_required(VERSION 3.5)
project(mbot_mapping)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include headers
include_directories(include)

add_library(moving_laser_scan
  src/moving_laser_scan.cpp
)

ament_target_dependencies(moving_laser_scan
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Build executable
add_executable(mapping_node
  src/mapping.cpp
  src/mapping_node.cpp
)

target_link_libraries(mapping_node
  moving_laser_scan
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Link dependencies
ament_target_dependencies(mapping_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

# Install executable
install(TARGETS mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS moving_laser_scan
  EXPORT  moving_laser_scanTargets
  DESTINATION lib
)

# Install headers (optional but good practice)
install(DIRECTORY include/
        DESTINATION include)

ament_export_targets(moving_laser_scanTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
