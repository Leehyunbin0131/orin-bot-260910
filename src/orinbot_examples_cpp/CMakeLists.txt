cmake_minimum_required(VERSION 3.8)
project(orinbot_examples_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(depth_safety_filter src/depth_safety_filter.cpp)
ament_target_dependencies(depth_safety_filter
  rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS depth_safety_filter
  DESTINATION lib/${PROJECT_NAME})

ament_package()
