cmake_minimum_required(VERSION 3.8)
project(amm_task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(amm_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)

include_directories(include)

add_executable(task_planner_node
  src/task_planner_node.cpp
)

target_include_directories(task_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(task_planner_node
  rclcpp
  rclcpp_action
  amm_msgs
  std_msgs
)

target_link_libraries(task_planner_node
  CURL::libcurl
)

install(TARGETS task_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
