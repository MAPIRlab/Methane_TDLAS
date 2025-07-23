cmake_minimum_required(VERSION 3.11)
project(falcon_tdlas)


##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")



##################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(serial REQUIRED)

##################
include_directories(include)

set(SOURCES_FILES 
	src/falcon.cpp
)


##################
add_executable(falcon_tdlas  ${SOURCES_FILES})

ament_target_dependencies(falcon_tdlas
  rclcpp
  olfaction_msgs
  serial
)

install(
  TARGETS falcon_tdlas
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()