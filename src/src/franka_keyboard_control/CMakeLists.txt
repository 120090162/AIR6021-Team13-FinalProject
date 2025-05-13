cmake_minimum_required(VERSION 3.8)
project(franka_keyboard_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
#find_package(rm_simulation2_moveit_config REQUIRED)
find_package(rm_moveit_config REQUIRED)
find_package(robot_arm_simulation REQUIRED)
#find_package(moveit_resources_panda_moveit_config REQUIRED)
#find_package(moveit_resources_panda_description REQUIRED)
ament_auto_find_build_dependencies()

# Add the keyboard control executable
add_executable(rm_servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(rm_servo_keyboard_input PUBLIC include)
ament_target_dependencies(rm_servo_keyboard_input std_msgs control_msgs rclcpp rclcpp_action geometry_msgs sensor_msgs)

add_executable(rm_servo_cpp_interface src/servo_cpp_interface.cpp)
target_include_directories(rm_servo_cpp_interface PUBLIC include)
ament_target_dependencies(rm_servo_cpp_interface moveit_servo rclcpp)

# Install executables
install(TARGETS
  rm_servo_keyboard_input
  rm_servo_cpp_interface
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

# Install config files
#install(DIRECTORY config
#  DESTINATION share/${PROJECT_NAME}
#)

#ament_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/control_manager.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN joint_state_converter::JointStateToRmCmdNode
  EXECUTABLE joint_state_to_rm_cmd
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)