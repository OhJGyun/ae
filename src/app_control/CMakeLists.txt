cmake_minimum_required(VERSION 3.8)
project(app_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pure_pursuit_sim_node src/pure_pursuit_sim_node.cpp)
ament_target_dependencies(pure_pursuit_sim_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  ackermann_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(stanley_node src/stanley_node.cpp)
ament_target_dependencies(stanley_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  ackermann_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS pure_pursuit_sim_node stanley_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()