cmake_minimum_required(VERSION 3.8)
project(amm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes OPTIONAL)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
