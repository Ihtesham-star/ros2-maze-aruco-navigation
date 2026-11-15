cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# maze_explorer_node (wall follower - no Nav2 needed)
add_executable(maze_explorer_node src/maze_explorer_node.cpp)
ament_target_dependencies(maze_explorer_node
  rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs ament_index_cpp)
target_link_libraries(maze_explorer_node ${OpenCV_LIBS})

# map_saver_node
add_executable(map_saver_node src/map_saver_node.cpp)
ament_target_dependencies(map_saver_node
  rclcpp std_msgs nav_msgs)

# aruco_goal_node (Phase 2 - uses Nav2)
add_executable(aruco_goal_node src/aruco_goal_node.cpp)
ament_target_dependencies(aruco_goal_node
  rclcpp rclcpp_action std_msgs geometry_msgs nav2_msgs ament_index_cpp)
target_link_libraries(aruco_goal_node yaml-cpp)

# Install executables
install(TARGETS
  maze_explorer_node
  map_saver_node
  aruco_goal_node
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY
  launch config worlds urdf meshes materials maps
  DESTINATION share/${PROJECT_NAME})

ament_package()
