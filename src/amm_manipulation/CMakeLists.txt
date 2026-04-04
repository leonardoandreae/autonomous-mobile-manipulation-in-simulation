cmake_minimum_required(VERSION 3.8)
project(amm_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amm_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(manipulation_manager_node
  src/manipulation_manager_node.cpp
)

ament_target_dependencies(manipulation_manager_node
  rclcpp
  rclcpp_action
  amm_msgs
  moveit_ros_planning_interface
  moveit_task_constructor_core
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS manipulation_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/   DESTINATION share/${PROJECT_NAME}/config OPTIONAL)

ament_package()
