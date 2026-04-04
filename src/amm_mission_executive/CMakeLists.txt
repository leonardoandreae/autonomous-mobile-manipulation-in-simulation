cmake_minimum_required(VERSION 3.8)
project(amm_mission_executive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amm_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(mission_executive_node
  src/mission_executive_node.cpp
)

ament_target_dependencies(mission_executive_node
  rclcpp
  rclcpp_action
  amm_msgs
  std_msgs
  geometry_msgs
  nav2_msgs
)

install(TARGETS mission_executive_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
