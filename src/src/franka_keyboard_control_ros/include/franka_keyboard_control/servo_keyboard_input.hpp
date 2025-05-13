#ifndef SERVO__RM_SERIAL_DRIVER_HPP_
#define SERVO__RM_SERIAL_DRIVER_HPP_

#include "franka_keyboard_control/crc.hpp"
#include "franka_keyboard_control/packet.hpp"
#include "franka_keyboard_control/serial_driver.hpp"
// ros2
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <thread>
#include <vector>

#include "rm_utils/heartbeat.hpp"

using namespace fyt;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

namespace ext_serial_driver
{
    class KeyboardServo : public rclcpp::Node {
        public:
            // 话题和帧配置
            std::string TWIST_TOPIC = "/servo_demo_node/delta_twist_cmds";
            std::string JOINT_TOPIC = "/servo_demo_node/delta_joint_cmds";
            size_t ROS_QUEUE_SIZE = 10;
            std::vector<std::string> frame;
            std::string IMAGE_FRAME_ID = "link3";
            std::string BASE_FRAME_ID = "base_link";

            explicit KeyboardServo(const rclcpp::NodeOptions &options);
        
        private:
            enum ControlMode { TRANSLATION, ROTATION };
            ControlMode current_mode_ = TRANSLATION;
            // Serial port
            std::unique_ptr<Port> port_;
            
            rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
            rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
            
            std::string frame_to_publish_ = BASE_FRAME_ID;
            double vel_cmd_;
            // Heartbeat
            HeartBeatPublisher::SharedPtr heartbeat_;

            void send_home_goal(const std::vector<double>& positions);
            void keyLoop();
            void process_key(uint8_t c);
            void receiveData();
            void sendData(const sensor_msgs::msg::JointState::SharedPtr msg);
        };
}

#endif // UP_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_