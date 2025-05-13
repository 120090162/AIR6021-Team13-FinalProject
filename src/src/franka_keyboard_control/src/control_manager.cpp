#include "franka_keyboard_control/control_manager.hpp"

#include <chrono>
#include <vector>
#include <string>

namespace joint_state_converter
{

    JointStateToRmCmdNode::JointStateToRmCmdNode(const rclcpp::NodeOptions &options)
        : Node("joint_state_to_rm_cmd", options)
    {
        RCLCPP_INFO(get_logger(), "Start JointStateToRmCmdNode!");

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointStateToRmCmdNode::jointStateCallback, this, std::placeholders::_1));

        rm_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/rm_cmd", 10);

        // Initialize state with 7 joints (joint0 to joint6)
        state_.positions.resize(7, 0.0); // Initialize with zeros for 7 joints

        // Timer to periodically check and log (optional, for consistency with GimbalManagerNode)
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]()
                                   { RCLCPP_DEBUG(get_logger(), "Node running, last joint positions: %s",
                                                  jointPositionsToString().c_str()); });
    }

    void JointStateToRmCmdNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Expected joint names in order: joint0 to joint6
        std::vector<std::string> expected_joints = {
            "joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        // Temporary storage for new positions
        std::vector<double> new_positions(7, 0.0);

        // Map joint names to positions
        bool valid_message = true;
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
        {
            auto it = std::find(expected_joints.begin(), expected_joints.end(), msg->name[i]);
            if (it != expected_joints.end())
            {
                size_t index = std::distance(expected_joints.begin(), it);
                new_positions[index] = msg->position[i];
            }
            else
            {
                RCLCPP_WARN(get_logger(), "Unexpected joint name: %s", msg->name[i].c_str());
                valid_message = false;
            }
        }

        // Check if all expected joints were found
        if (msg->name.size() < expected_joints.size())
        {
            RCLCPP_WARN(get_logger(), "Received JointState with fewer joints than expected (%zu < %zu)",
                        msg->name.size(), expected_joints.size());
            valid_message = false;
        }

        // Update state and publish only if message is valid
        if (valid_message)
        {
            state_.positions = new_positions;
            publishRmCmd();
        }
    }

    void JointStateToRmCmdNode::publishRmCmd()
    {
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = state_.positions; // Assign positions in order: joint0 to joint6
        rm_cmd_pub_->publish(msg);

        RCLCPP_DEBUG(get_logger(), "Published rm_cmd: %s", jointPositionsToString().c_str());
    }

    std::string JointStateToRmCmdNode::jointPositionsToString() const
    {
        std::stringstream ss;
        ss << "[";
        for (size_t i = 0; i < state_.positions.size(); ++i)
        {
            ss << state_.positions[i];
            if (i < state_.positions.size() - 1)
                ss << ", ";
        }
        ss << "]";
        return ss.str();
    }

} // namespace joint_state_converter

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(joint_state_converter::JointStateToRmCmdNode)