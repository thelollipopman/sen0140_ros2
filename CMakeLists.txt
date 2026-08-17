cmake_minimum_required(VERSION 3.20)
project(sen0140_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Sensor driver library
add_library(sen0140_sensors
  src/vcm5883l.cpp
  src/bmp280.cpp
  src/adxl345.cpp
  src/itg3200.cpp
)

target_include_directories(sen0140_sensors
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)


# ROS 2 node
add_executable(sen0140_node
  src/sen0140_node.cpp
)

target_link_libraries(sen0140_node
  sen0140_sensors
)

ament_target_dependencies(sen0140_node
  rclcpp
  sensor_msgs
)


# Install executable/library
install(
  TARGETS
    sen0140_sensors
    sen0140_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


# Install public headers
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
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
