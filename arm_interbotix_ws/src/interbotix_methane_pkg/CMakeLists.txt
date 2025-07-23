cmake_minimum_required(VERSION 3.8)
project(interbotix_methane_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(olfaction_msgs REQUIRED)

include_directories(
  include 
)

add_executable(scanner_ptu src/scanner_ptu.cpp)
ament_target_dependencies(scanner_ptu
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  interbotix_xs_msgs)

add_executable(scanner_ptu_2 src/scanner_ptu_2.cpp)
ament_target_dependencies(scanner_ptu_2
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  interbotix_xs_msgs)

add_executable(scanner_ptu_3D src/scanner_ptu_3D.cpp)
ament_target_dependencies(scanner_ptu_3D
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
  interbotix_xs_msgs)

add_executable(methane_visualizer src/methane_visualizer.cpp)
ament_target_dependencies(methane_visualizer
  rclcpp
  visualization_msgs
  std_msgs
  sensor_msgs
  interbotix_xs_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  olfaction_msgs
)

add_executable(map_obstacles src/map_obstacles.cpp)
ament_target_dependencies(map_obstacles
  rclcpp
  moveit_ros_planning_interface
)
add_executable(move_arm src/move_arm.cpp)
ament_target_dependencies(move_arm
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)
install(TARGETS
  map_obstacles
  move_arm
  scanner_ptu
  scanner_ptu_2
  scanner_ptu_3D
  methane_visualizer
  DESTINATION lib/${PROJECT_NAME}
)

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
