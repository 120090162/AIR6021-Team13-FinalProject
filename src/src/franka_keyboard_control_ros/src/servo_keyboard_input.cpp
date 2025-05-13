#include "franka_keyboard_control/servo_keyboard_input.hpp"

// 键盘键位定义
#define KEYCODE_LEFT   0x44   // 左方向键
#define KEYCODE_RIGHT  0x43   // 右方向键
#define KEYCODE_A      0x61   // a
#define KEYCODE_S      0x73   // s
#define KEYCODE_D      0x64   // d
#define KEYCODE_Q      0x71   // q
#define KEYCODE_W      0x77   // w
#define KEYCODE_E      0x65   // e
#define KEYCODE_R      0x72   // r
#define KEYCODE_F      0x66   // f
#define KEYCODE_Z      0x7A   // z
#define KEYCODE_X      0x78   // x
#define KEYCODE_C      0x63   // c
#define KEYCODE_SPACE  0x20   // 空格键退出

namespace ext_serial_driver
{
    KeyboardServo::KeyboardServo(const rclcpp::NodeOptions &options)
        : Node("keyboard_servo_node", options), port_{new Port(2)}, vel_cmd_(1.0)
    {
        RCLCPP_INFO(get_logger(), "Start KeyboardServo!");
        port_->getParams("/dev/ttyACM0", 115200, "none", "none", "1");

        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/manipulator_controller/follow_joint_trajectory");

        try
        {
            port_->serial_driver_->init_port(port_->device_name_, *port_->device_config_);
            if (!port_->serial_driver_->port()->is_open())
            {
                port_->serial_driver_->port()->open();
                port_->receive_thread_ = std::thread(&KeyboardServo::receiveData, this);
                // LOG
                RCLCPP_INFO(get_logger(), "serial open OK!");
            }
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(
                get_logger(), "Error creating serial port: %s - %s", port_->device_name_.c_str(), ex.what());
            throw ex;
        }

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 100,
            std::bind(&KeyboardServo::sendData, this, std::placeholders::_1));
        // Heartbeat
        heartbeat_ = HeartBeatPublisher::create(this);
    }

    void KeyboardServo::receiveData()
    {
        std::vector<uint8_t> header(1); // unsigned int (16)
        std::vector<uint8_t> data;
        data.reserve(sizeof(UpReceivePacket));

        while (rclcpp::ok())
        {
            try
            {
                // RCLCPP_INFO(get_logger(), "[Receive] receive_header %u!", header[0]);
                port_->serial_driver_->port()->receive(header);
                if (header[0] == 0xA8)
                {
                    data.resize(sizeof(UpReceivePacket) - 1);
                    port_->serial_driver_->port()->receive(data);
                    data.insert(data.begin(), header[0]);
                    UpReceivePacket packet = fromVector<UpReceivePacket>(data);

                    bool crc_ok =
                        crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
                    if (crc_ok)
                    {
                        // RCLCPP_INFO(get_logger(), "CRC OK!");
                        uint8_t c = packet.key_code;
                        process_key(c);
                    }
                    else
                    {
                        RCLCPP_ERROR(get_logger(), "CRC error!");
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
                }
            }
            catch (const std::exception &ex)
            {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
                port_->reopenPort();
            }
        }
    }

    void KeyboardServo::send_home_goal(const std::vector<double>& positions) {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(get_logger(), "动作服务器不可用");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {
            "joint0", "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.sec = 2;
        
        goal_msg.trajectory.points.push_back(point);
        
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandle::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(get_logger(), "目标被拒绝");
                } else {
                    RCLCPP_INFO(get_logger(), "目标已接受");
                }
            };
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void KeyboardServo::process_key(uint8_t c) {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        bool publish_twist = false;

        switch (c) {
        // 模式切换
        case KEYCODE_LEFT:
            current_mode_ = TRANSLATION;
            RCLCPP_INFO(get_logger(), "切换到平移模式");
            break;
        case KEYCODE_RIGHT:
            current_mode_ = ROTATION;
            RCLCPP_INFO(get_logger(), "切换到旋转模式");
            break;

        // 运动控制
        case KEYCODE_W:
            if (current_mode_ == TRANSLATION) {
                twist_msg->twist.linear.x = 1.0 * vel_cmd_;
            } else {
                twist_msg->twist.angular.y = 1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_S:
            if (current_mode_ == TRANSLATION) {
                twist_msg->twist.linear.x = -1.0 * vel_cmd_;
            } else {
                twist_msg->twist.angular.y = -1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_A:
            if (current_mode_ == TRANSLATION) {
                twist_msg->twist.linear.y = -1.0 * vel_cmd_;
            } else {
                twist_msg->twist.angular.z = 1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_D:
            if (current_mode_ == TRANSLATION) {
                twist_msg->twist.linear.y = 1.0 * vel_cmd_;
            } else {
                twist_msg->twist.angular.z = -1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_Q:
            if (current_mode_ == TRANSLATION) {
                twist_msg->twist.linear.z = 1.0 * vel_cmd_;
            } else {
                twist_msg->twist.angular.x = 1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_E:
            if (current_mode_ == TRANSLATION) {
                twist_msg->twist.linear.z = -1.0 * vel_cmd_;
            } else {
                twist_msg->twist.angular.x = -1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;

        // 坐标系切换
        case KEYCODE_R:
            frame_to_publish_ = BASE_FRAME_ID;
            RCLCPP_INFO(get_logger(), "切换至世界坐标系");
            break;
        case KEYCODE_F:
            frame_to_publish_ = IMAGE_FRAME_ID;
            RCLCPP_INFO(get_logger(), "切换至图像坐标系");
            break;

        // Home位置
        case KEYCODE_Z:
            RCLCPP_INFO(get_logger(), "开车位置");
            send_home_goal({0.0, 0.0, 0.10467, -0.52333, 0.0, 0.0, 0.0});
            break;
        case KEYCODE_X:
            RCLCPP_INFO(get_logger(), "取矿位置");
            send_home_goal({3.1340207886607385e-05, -7.939978110037644e-05, -1.3053444405993753, -0.1801418865506103,
                            6.852515749897528e-07, -0.37716225203124915, -4.938359390554933e-05});
            break;
        case KEYCODE_C:
            RCLCPP_INFO(get_logger(), "兑矿位置");
            send_home_goal({-0.0006873558818435402, 0.0018702871837408188, -0.15121657114016726, 0.500740208883582, 
                            0.0016983079260446971, -0.7681844522909588, 0.0013878119294897257});
            break;

        if (publish_twist) {
            twist_msg->header.stamp = now();
            twist_msg->header.frame_id = frame_to_publish_;
            twist_pub_->publish(std::move(twist_msg));
        }
    }
    }

    void KeyboardServo::sendData(const sensor_msgs::msg::JointState::SharedPtr msg){
        if (msg->name.size() != msg->position.size()) {
            RCLCPP_ERROR(
              get_logger(), "JointState message name and position arrays are of different sizes");
            return;
        }
        try
        {
            SendPacket packet;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == "joint0") {
                    packet.joint0_state = msg->position[i];
                } else if (msg->name[i] == "joint1") {
                    packet.joint1_state = msg->position[i];
                } else if (msg->name[i] == "joint2") {
                    packet.joint2_state = msg->position[i];
                } else if (msg->name[i] == "joint3") {
                    packet.joint3_state = msg->position[i];
                } else if (msg->name[i] == "joint4") {
                    packet.joint4_state = msg->position[i];
                } else if (msg->name[i] == "joint5") {
                    packet.joint5_state = msg->position[i];
                } else if (msg->name[i] == "joint6") {
                    packet.joint6_state = msg->position[i];
                }
            }
  
              crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
              std::vector<uint8_t> data = toVector(packet);
              port_->serial_driver_->port()->send(data);
        }
        catch (const std::exception &ex)
        {
              RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
              port_->reopenPort();
        }
    }
};

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ext_serial_driver::KeyboardServo)
