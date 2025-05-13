import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder


def generate_launch_description():
    # moveit_config = (
    #     MoveItConfigsBuilder("rm_simulation2")
    #     .robot_description(file_path="config/robot_arm_simulation.urdf.xacro")
    #     .robot_description_semantic(file_path="config/robot_arm_simulation.srdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .to_moveit_configs()
    # )
    moveit_config = (
        MoveItConfigsBuilder("rm")
        .robot_description(file_path="config/robot_arm_simulation.urdf.xacro")
        .robot_description_semantic(file_path="config/robot_arm_simulation.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # Get parameters for the Servo node
    servo_params = (
        ParameterBuilder("franka_keyboard_control")
        .yaml(
            parameter_namespace="moveit_servo",
            file_path="config/rm_simulated_config.yaml",
        )
        .to_dict()
    )

    # A node to publish world -> panda_link0 transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # The servo cpp interface demo
    # Creates the Servo node and publishes commands to it
    servo_node = Node(
        package="franka_keyboard_control",
        executable="rm_servo_cpp_interface",
        output="screen",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Publishes tf's for the robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # RViz
    rviz_base = os.path.join(
        get_package_share_directory("franka_keyboard_control"), "config"
    )
    rviz_config_file = os.path.join(rviz_base, "moveit_empty.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                get_package_share_directory("rm_moveit_config"),
                "config",
                "ros2_controllers.yaml",
            ),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in ["manipulator_controller", "joint_state_broadcaster"]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [rviz_node, static_tf, servo_node, ros2_control_node, robot_state_publisher]
        + load_controllers
    )
