cmake_minimum_required(VERSION 3.5)
project(minirae_lite)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(serial REQUIRED)



include_directories(include)

add_executable(minirae_lite_node src/minirae_lite_node.cpp)

ament_target_dependencies(minirae_lite_node
  rclcpp
  olfaction_msgs
  serial  
)

add_executable(miniraeLiteID src/identifier.cpp)
ament_target_dependencies(miniraeLiteID serial)


install(
  TARGETS minirae_lite_node miniraeLiteID
  DESTINATION lib/${PROJECT_NAME}
)


install(
    FILES resources/10-minirae_lite.rules
    DESTINATION share/${PROJECT_NAME}
)

ament_package()