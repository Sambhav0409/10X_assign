cmake_minimum_required(VERSION 3.5)
project(turtlebot3_path_follow_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
ament_target_dependencies(waypoint_publisher rclcpp geometry_msgs std_msgs)

add_executable(path_smoother src/path_smoother.cpp)
ament_target_dependencies(path_smoother rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs)

add_executable(trajectory_generator src/trajectory_generator.cpp)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs nav_msgs std_msgs)

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp)
ament_target_dependencies(pure_pursuit_controller
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros visualization_msgs)

install(
  TARGETS
    waypoint_publisher
    path_smoother
    trajectory_generator
    pure_pursuit_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- ADD THIS BLOCK ---
# Install world files
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
# --- END OF BLOCK ---

ament_package()
