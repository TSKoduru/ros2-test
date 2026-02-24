cmake_minimum_required(VERSION 3.8)
project(mbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# localization_node
add_executable(localization_node
  src/action_model.cpp
  src/sensor_model.cpp
  src/particle_filter.cpp
  src/localization_node.cpp
  src/obstacle_distance_grid.cpp
)
ament_target_dependencies(localization_node
  rclcpp sensor_msgs nav_msgs nav2_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

# Install
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(TARGETS localization_node RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
