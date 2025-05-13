import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("franka_keyboard_control_ros")

    use_respawn = LaunchConfiguration("use_respawn")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="True",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    # Specify the actions
    start_franka_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir, "launch", "franka_interface.launch.py"
            )
        ),
    )

    record_rosbag_cmd = Node(
        package="franka_keyboard_control",
        executable="keyboard_servo_node",
        name="keyboard_servo_node",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_franka_cmd)
    ld.add_action(record_rosbag_cmd)

    return ld
