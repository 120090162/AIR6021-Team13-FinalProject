cmake_minimum_required(VERSION 3.10.2)
project(robot_arm_simulation)
find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf xml DESTINATION share/${PROJECT_NAME})
