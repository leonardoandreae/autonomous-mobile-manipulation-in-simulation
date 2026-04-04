cmake_minimum_required(VERSION 3.8)
project(amm_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(object_detector_node
  src/object_detector_node.cpp
)

ament_target_dependencies(object_detector_node
  rclcpp
  rclcpp_action
  amm_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS object_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config OPTIONAL)

ament_package()
