import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_respawn = LaunchConfiguration("use_respawn")

    directory = get_package_share_directory("robot_arm_simulation")
    xmlScenePath = os.path.join(directory, "xml", "scene.xml")

    if not os.path.exists(xmlScenePath):
        raise FileNotFoundError(f"Scene file does not exist: {xmlScenePath}.")

    start_mujoco_cmd = Node(
        package="mujoco_ros2",
        executable="mujoco_node",
        name="mujoco_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=[xmlScenePath],
        parameters=[
            {"joint_state_topic_name": "rm_state"},
            {"joint_command_topic_name": "rm_cmd"},
            {"enable_command_topic_name": "enable_cmd"},
            {"control_mode": "POSITION"},
            {"simulation_frequency": 1000},
            {"visualisation_frequency": 20},
            {"mass": 1000.0},
            # {"camera_focal_point": [0.0, 0.0, 0.25]},
            # {"camera_distance": 2.5},
            # {"camera_azimuth": 135.0},
            # {"camera_elevation": -20.0},
            # {"camera_orthographic": True},
        ],
    )

    start_manager_cmd = Node(
        package="franka_keyboard_control",
        executable="joint_state_to_rm_cmd",
        name="joint_state_to_rm_cmd",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    ld = LaunchDescription()

    ld.add_action(start_mujoco_cmd)
    ld.add_action(start_manager_cmd)

    return ld
