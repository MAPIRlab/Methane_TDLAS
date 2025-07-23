cmake_minimum_required(VERSION 3.5)
project(senseair)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(serial REQUIRED)



include_directories(include)

add_executable(senseair_node src/senseair.cpp)

ament_target_dependencies(senseair_node
  rclcpp
  olfaction_msgs
  serial  
)

install(
  TARGETS senseair_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()