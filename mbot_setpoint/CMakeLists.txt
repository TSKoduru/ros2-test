cmake_minimum_required(VERSION 3.8)
project(mbot_setpoint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mbot_interfaces REQUIRED)

include_directories(include)

# === C++ Executables ===

# --- motion_controller_diff ---
add_executable(motion_controller_diff src/motion_controller_diff.cpp)
ament_target_dependencies(motion_controller_diff rclcpp geometry_msgs nav_msgs std_msgs mbot_interfaces tf2_ros tf2_geometry_msgs)
install(TARGETS motion_controller_diff DESTINATION lib/${PROJECT_NAME})

# --- square_publisher ---
add_executable(square_publisher src/square_publisher.cpp)
ament_target_dependencies(square_publisher rclcpp geometry_msgs std_msgs mbot_interfaces)
install(TARGETS square_publisher DESTINATION lib/${PROJECT_NAME})

# --- maze publisher ---
add_executable(maze_publisher src/maze_publisher.cpp)
ament_target_dependencies(maze_publisher rclcpp geometry_msgs std_msgs mbot_interfaces)
install(TARGETS maze_publisher DESTINATION lib/${PROJECT_NAME})

# --- odom ---
add_executable(odom src/odom.cpp)
ament_target_dependencies(odom rclcpp geometry_msgs nav_msgs tf2_msgs mbot_interfaces)
install(TARGETS odom DESTINATION lib/${PROJECT_NAME})

ament_package()
