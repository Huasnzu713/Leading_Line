cmake_minimum_required(VERSION 3.16)
project(leading_line_chassis LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --- 依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# libserial-dev：apt 装的串口库
find_path(SERIAL_INCLUDE_DIR serial/serial.h)
find_library(SERIAL_LIBRARY NAMES serial)
if(NOT SERIAL_INCLUDE_DIR OR NOT SERIAL_LIBRARY)
  message(FATAL_ERROR "libserial-dev not found. apt install libserial-dev")
endif()
message(STATUS "libserial: include=${SERIAL_INCLUDE_DIR} lib=${SERIAL_LIBRARY}")

# --- 头文件目录 ---
add_library(leading_line_chassis_core STATIC
  src/protocol.cpp
  src/serial_driver.cpp
  src/ackermann_kinematics.cpp
)
target_include_directories(leading_line_chassis_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${SERIAL_INCLUDE_DIR}
)
target_link_libraries(leading_line_chassis_core PUBLIC ${SERIAL_LIBRARY})
ament_target_dependencies(leading_line_chassis_core
  rclcpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# --- 主节点可执行 ---
add_executable(chassis_node src/chassis_node.cpp)
target_link_libraries(chassis_node PRIVATE leading_line_chassis_core)
ament_target_dependencies(chassis_node
  rclcpp
  ackermann_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)

# --- 安装 ---
install(TARGETS chassis_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

# --- 测试 ---
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_chassis_protocol test/test_chassis_protocol.cpp)
  target_link_libraries(test_chassis_protocol PRIVATE leading_line_chassis_core)
endif()

ament_package()
