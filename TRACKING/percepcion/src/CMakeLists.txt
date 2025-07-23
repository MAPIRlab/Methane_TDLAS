cmake_minimum_required(VERSION 3.8)
project(percepcion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")
find_package(OpenCV REQUIRED)
message(STATUS "Versión de OpenCV encontrada: ${OpenCV_VERSION}")
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(interbotix_xs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/GeographicLib")
#find_package(GeographicLib REQUIRED)

include_directories(
  include 
  ${OpenCV_INCLUDE_DIRS}  # Añade esto para incluir los directorios de OpenCV
  ${nlohmann_json_INCLUDE_DIRS}
)
# Add the executable
add_executable(detector_aruco src/detector_aruco.cpp)
add_executable(mov_PTU src/mov_PTU.cpp)
target_link_libraries(mov_PTU Geographic)
add_executable(medir_retraso src/medir_retraso.cpp)
add_executable(simulacion_movimiento src/simulacion_movimiento.cpp)

# Link dependencies
ament_target_dependencies(detector_aruco 
  rclcpp 
  sensor_msgs 
  image_transport 
  cv_bridge 
  OpenCV
  nlohmann_json
  std_msgs
)

ament_target_dependencies(mov_PTU 
  rclcpp 
  std_msgs
  interbotix_xs_msgs
  nlohmann_json
  sensor_msgs
  Eigen3
)

ament_target_dependencies(medir_retraso
  rclcpp
  std_msgs
  interbotix_xs_msgs
  sensor_msgs
)

ament_target_dependencies(simulacion_movimiento
  rclcpp 
  nlohmann_json
  std_msgs
  sensor_msgs
)

# Install the executable
install(TARGETS detector_aruco mov_PTU medir_retraso simulacion_movimiento DESTINATION lib/${PROJECT_NAME})


# -------------------------------------------------
# Instalar carpeta de parámetros (config/)
# -------------------------------------------------
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

# -------------------------------------------------
# Instalar carpeta de launch files (launch/)
# -------------------------------------------------
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Testing and linting setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Skips the copyright check
  set(ament_cmake_cpplint_FOUND TRUE)    # Skips cpplint
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

