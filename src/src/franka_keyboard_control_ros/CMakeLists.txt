cmake_minimum_required(VERSION 3.8)
project(franka_keyboard_control_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_moveit_config REQUIRED)
find_package(robot_arm_simulation REQUIRED)
ament_auto_find_build_dependencies()

add_executable(rm_servo_cpp_interface src/servo_cpp_interface.cpp)
target_include_directories(rm_servo_cpp_interface PUBLIC include)
ament_target_dependencies(rm_servo_cpp_interface moveit_servo rclcpp)

install(TARGETS
  rm_servo_cpp_interface
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/serial_driver.cpp
  src/servo_keyboard_input.cpp
  src/crc.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ext_serial_driver::KeyboardServo
  EXECUTABLE keyboard_servo_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)