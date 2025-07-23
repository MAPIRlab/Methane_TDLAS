cmake_minimum_required(VERSION 3.8)
project(hunter_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(olfaction_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include 
  ${nlohmann_json_INCLUDE_DIRS}
)

install(DIRECTORY 
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

add_executable(navigation_through_poses src/navigation_through_poses.cpp)
ament_target_dependencies(navigation_through_poses
  rclcpp
  geometry_msgs
  nlohmann_json
  std_msgs
  nav2_msgs
  rclcpp_action
  olfaction_msgs
)
install(TARGETS
  navigation_through_poses
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
