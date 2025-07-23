cmake_minimum_required(VERSION 3.5)
project(windsonic)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)



include_directories(include)

add_executable(windsonic_node src/windsonic.cpp)

ament_target_dependencies(windsonic_node
  rclcpp
  olfaction_msgs
  serial  
  tf2
  tf2_geometry_msgs
  angles
  geometry_msgs
  visualization_msgs
)

add_executable(windsonicID src/identifier.cpp)
ament_target_dependencies(windsonicID serial)

install(
  TARGETS windsonic_node windsonicID
  DESTINATION lib/${PROJECT_NAME}
)

install(
    FILES resources/10-windsonic.rules
    DESTINATION share/${PROJECT_NAME}
)

ament_package()