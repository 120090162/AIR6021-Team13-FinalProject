#ifndef JOINT_STATE_CONVERTER__JOINT_STATE_TO_RM_CMD_NODE_HPP_
#define JOINT_STATE_CONVERTER__JOINT_STATE_TO_RM_CMD_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace joint_state_converter
{

    class JointStateToRmCmdNode : public rclcpp::Node
    {
    public:
        explicit JointStateToRmCmdNode(const rclcpp::NodeOptions &options);

    private:
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void publishRmCmd();
        std::string jointPositionsToString() const;

        // State to store joint positions (joint0 to joint6)
        struct State
        {
            std::vector<double> positions; // Positions for joint0 to joint6
        } state_;

        // Subscriptions and publishers
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rm_cmd_pub_;

        // Timer for periodic checks (optional)
        rclcpp::TimerBase::SharedPtr timer_;
    };

} // namespace joint_state_converter

#endif // JOINT_STATE_CONVERTER__JOINT_STATE_TO_RM_CMD_NODE_HPP_