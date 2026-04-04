cmake_minimum_required(VERSION 3.8)
project(amm_slam_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_executable(slam_manager_node
  src/slam_manager_node.cpp
)

ament_target_dependencies(slam_manager_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  nav_msgs
  std_srvs
)

install(TARGETS slam_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
