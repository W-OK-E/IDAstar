cmake_minimum_required(VERSION 3.8)
project(cruise)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(cruise src/ida.cpp)
add_executable(dynamic_map_publisher src/dynamic_maps.cpp)


ament_target_dependencies(cruise
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(dynamic_map_publisher
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  cruise
  dynamic_map_publisher
  DESTINATION lib/${PROJECT_NAME})


ament_package()
