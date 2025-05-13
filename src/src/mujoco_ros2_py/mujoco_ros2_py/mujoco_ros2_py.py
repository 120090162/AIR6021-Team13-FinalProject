import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Bool
from datetime import datetime
import csv

import mujoco
import mujoco.glfw
import mujoco.viewer
import time
from threading import Thread
import threading


class MuJoCoROS(Node):
    def __init__(self, xml_path):
        super().__init__("mujoco_node")
        # Placeholder for MuJoCo initialization or other node setup
        self.get_logger().info(f"Initializing MuJoCoROS with XML path: {xml_path}")

        # Declare & get parameters for this node
        self.declare_parameter("joint_state_topic_name", "joint_state")
        self.declare_parameter("joint_command_topic_name", "joint_commands")
        self.declare_parameter("enable_command_topic_name", "enable_command")
        self.declare_parameter("control_mode", "TORQUE")
        self.declare_parameter("simulation_frequency", 1000)
        self.declare_parameter("visualisation_frequency", 20)
        self.declare_parameter("mass", 1.0)
        self.declare_parameter("csv_filename", "mujoco_data")
        self.declare_parameter("is_csv_logger", False)

        joint_state_topic_name = self.get_parameter("joint_state_topic_name").value
        joint_command_topic_name = self.get_parameter("joint_command_topic_name").value
        enable_command_topic_name = self.get_parameter(
            "enable_command_topic_name"
        ).value
        self.control_mode = self.get_parameter("control_mode").value
        self.sim_frequency = self.get_parameter("simulation_frequency").value
        self.visualisation_dt = (
            1.0 / self.get_parameter("visualisation_frequency").value
        )
        self.mass = self.get_parameter("mass").value
        self.csv_filename = self.get_parameter("csv_filename").value
        self.get_logger().info(f"CSV filename: {self.csv_filename}")
        self.is_csv_logger = self.get_parameter("is_csv_logger").value

        if self.is_csv_logger:
            self.CSVInit()

        # Load the robot model
        self.locker = threading.Lock()
        try:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        except Exception as e:
            self.get_logger().error(f"Problem loading model: {e}")
            sys.exit(1)
        self.ctrl_data = [0.0] * self.model.nu

        self.model.opt.timestep = (
            1.0 / self.sim_frequency
        )  # Match MuJoCo to node frequency

        # Initialize joint state
        self.data = mujoco.MjData(self.model)

        # Resize arrays based on the number of joints in the model
        self.joint_state_message = JointState()
        self.joint_state_message.name = [
            self.model.joint(i).name for i in range(self.model.nu)
        ]
        self.joint_state_message.position = [0.0] * self.model.nu
        self.joint_state_message.velocity = [0.0] * self.model.nu
        self.joint_state_message.effort = [0.0] * self.model.nu
        self.torque_input = [0.0] * self.model.nu

        self.viewer = mujoco.viewer.launch_passive(
            self.model, self.data, key_callback=self.MujocoKeyCallback
        )

        # Create joint state publisher and joint command subscriber
        self.joint_command_subscriber = self.create_subscription(
            Float64MultiArray, joint_command_topic_name, self.joint_command_callback, 1
        )
        self.pick_place_command_subscriber = self.create_subscription(
            Bool, enable_command_topic_name, self.pick_place_command_callback, 1
        )
        self.joint_state_publisher = self.create_publisher(
            JointState, joint_state_topic_name, 1
        )

        # Initialize visualization (simplified for Python)
        self.get_logger().info("MuJoCo simulation initiated.")
        self.get_logger().info(
            "Publishing the joint state to '%s' topic. " % (joint_state_topic_name)
        )
        self.get_logger().info(
            "Subscribing to joint %s commands via '%s' topic."
            % (self.control_mode, joint_command_topic_name)
        )

        # Get gripper body ID
        self.gripper_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "link6"
        )
        if self.gripper_body_id == -1:
            self.get_logger().error("Could not find gripper.")
            sys.exit(1)

        self.is_picking = False

        # thread loops
        self.viewer_thread = Thread(target=self.PhysicsViewerThread)
        self.sim_thread = Thread(target=self.SimulationThread)
        self.viewer_thread.start()
        self.sim_thread.start()

    def update_simulation(self):
        self.data.ctrl = self.ctrl_data

        mujoco.mj_step(self.model, self.data)

        # Update joint state message
        for i in range(self.model.nu):
            self.joint_state_message.position[i] = self.data.sensordata[i]
            self.joint_state_message.velocity[i] = self.data.sensordata[
                i + self.model.nu
            ]
            self.joint_state_message.effort[i] = self.data.sensordata[
                i + 2 * self.model.nu
            ]

        self.joint_state_message.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_publisher.publish(self.joint_state_message)

    def joint_command_callback(self, msg):
        if len(msg.data) != self.model.nu:
            self.get_logger().warn("Received joint command with incorrect size.")
            return
        self.ctrl_data = msg.data

    def pick_place_command_callback(self, msg):
        self.locker.acquire()
        if msg.data and not self.is_picking:
            self.is_picking = True
            mujoco.mj_applyFT(
                self.model,
                self.data,
                [0, 0, -9.8 * self.mass],
                [0, 0, 0],
                self.data.xipos[self.gripper_body_id],
                self.gripper_body_id,
                self.data.qfrc_applied,
            )
        elif not msg.data and self.is_picking:
            self.is_picking = False
            mujoco.mju_zero(self.data.qfrc_applied)
        self.locker.release()

    def SimulationThread(self):
        # Close the viewer automatically after simulation_duration wall-seconds.
        while self.viewer.is_running() and rclpy.ok():
            step_start = time.perf_counter()
            self.locker.acquire()
            # physics update
            self.update_simulation()
            self.locker.release()

            if self.is_csv_logger:
                # Log the data to CSV
                self.CSVLogger(
                    self.data.time,
                    [self.data.qacc[i] for i in range(7)],
                    [self.data.sensordata[i + self.model.nu] for i in range(7)],
                    [self.data.sensordata[i + 2 * self.model.nu] for i in range(7)],
                )

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = self.model.opt.timestep - (
                time.perf_counter() - step_start
            )
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    def PhysicsViewerThread(self):
        while self.viewer.is_running() and rclpy.ok():
            self.locker.acquire()
            self.viewer.sync()
            self.locker.release()
            time.sleep(self.visualisation_dt)

    def MujocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        return

    def CSVInit(self):
        # Uncomment these lines if need timestamp for file name
        now = datetime.now()
        timestamp = now.strftime("%Y%m%d%H%M%S")
        self.csv_filename += f"/{timestamp}_data"

        self.csv_filename += ".csv"

        with open(self.csv_filename, "w", newline="") as file:
            writer = csv.writer(file)

            header = []
            header += ["time"]
            header += [f"joint_acc_{i}" for i in range(7)]
            header += [f"joint_vel_{i}" for i in range(7)]
            header += [f"joint_force_{i}" for i in range(7)]

            writer.writerow(header)

    def CSVLogger(self, time, acc, vel, force):
        with open(self.csv_filename, "a", newline="") as file:
            writer = csv.writer(file)

            row = []
            row.append(time)
            row += acc
            row += vel
            row += force

            writer.writerow(row)


def main(args=None):
    # Check for command-line argument
    if len(sys.argv) < 2:
        print(
            "[ERROR] Invalid number of arguments. Usage: mujoco_node path/to/scene.xml"
        )
        sys.exit(1)

    xml_path = sys.argv[1]

    # Initialize ROS2
    rclpy.init(args=args)

    try:
        # Create and spin the node
        node = MuJoCoROS(xml_path)
        rclpy.spin(node)

    except Exception as e:
        # Handle exceptions and log error
        print(f"[ERROR] {e}")
        sys.exit(1)

    finally:
        # Ensure proper shutdown
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
