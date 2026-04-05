cmake_minimum_required(VERSION 3.8)
project(amm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/          DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY robots/        DESTINATION share/${PROJECT_NAME}/robots)
install(DIRECTORY end_effectors/ DESTINATION share/${PROJECT_NAME}/end_effectors)
install(DIRECTORY meshes/        DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY launch/        DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/        DESTINATION share/${PROJECT_NAME}/config OPTIONAL)

ament_package()
