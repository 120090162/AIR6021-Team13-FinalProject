#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <memory>
#include <thread>

// 键盘键位定义
#define KEYCODE_LEFT 0x44  // 左方向键
#define KEYCODE_RIGHT 0x43 // 右方向键
#define KEYCODE_A 0x61     // a
#define KEYCODE_S 0x73     // s
#define KEYCODE_D 0x64     // d
#define KEYCODE_Q 0x71     // q
#define KEYCODE_W 0x77     // w
#define KEYCODE_E 0x65     // e
#define KEYCODE_R 0x72     // r
#define KEYCODE_F 0x66     // f
#define KEYCODE_Z 0x7A     // z
#define KEYCODE_X 0x78     // x
#define KEYCODE_C 0x63     // c
#define KEYCODE_J 0x6A     // j
#define KEYCODE_K 0x6B     // k
#define KEYCODE_SPACE 0x20 // 空格键退出

// 话题和帧配置
const std::string TWIST_TOPIC = "/servo_demo_node/delta_twist_cmds";
const std::string CMD_TOPIC = "/enable_cmd";
const std::string JOINT_TOPIC = "/servo_demo_node/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 10;
const std::string IMAGE_FRAME_ID = "link3";
const std::string BASE_FRAME_ID = "base_link";

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

class KeyboardReader
{
public:
    KeyboardReader() : kfd(0)
    {
        tcgetattr(kfd, &cooked);
        struct termios raw;
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
    }

    void readOne(char *c)
    {
        int rc = read(kfd, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
    }

    void shutdown()
    {
        tcsetattr(kfd, TCSANOW, &cooked);
    }

private:
    int kfd;
    struct termios cooked;
};

class KeyboardServo : public rclcpp::Node
{
public:
    KeyboardServo() : Node("keyboard_servo_node"), vel_cmd_(0.2)
    {
        twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
        cmd_pub_ = create_publisher<std_msgs::msg::Bool>(CMD_TOPIC, ROS_QUEUE_SIZE);
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this,
            "/manipulator_controller/follow_joint_trajectory");
    }

    void send_home_goal(const std::vector<double> &positions)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(), "动作服务器不可用");
            return;
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = {
            "joint0", "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"};

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = positions;
        point.time_from_start.sec = 5;

        goal_msg.trajectory.points.push_back(point);

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandle::SharedPtr &goal_handle)
        {
            if (!goal_handle)
            {
                RCLCPP_ERROR(get_logger(), "目标被拒绝");
            }
            else
            {
                RCLCPP_INFO(get_logger(), "目标已接受");
            }
        };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void keyLoop()
    {
        char c;
        KeyboardReader input;

        std::thread spin_thread([this]()
                                { rclcpp::spin(shared_from_this()); });

        print_instructions();

        while (rclcpp::ok())
        {
            try
            {
                input.readOne(&c);
            }
            catch (const std::runtime_error &e)
            {
                RCLCPP_ERROR(get_logger(), "读取键盘输入失败: %s", e.what());
                break;
            }

            process_key(c);
        }

        input.shutdown();
        spin_thread.join();
    }

private:
    enum ControlMode
    {
        TRANSLATION,
        ROTATION
    };
    ControlMode current_mode_ = TRANSLATION;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr cmd_pub_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    std::string frame_to_publish_ = BASE_FRAME_ID;
    double vel_cmd_;

    void print_instructions()
    {
        puts("\n=== 机械臂键盘控制 ===");
        puts("模式切换:");
        puts("  左方向键: 平移模式");
        puts("  右方向键: 旋转模式");
        puts("平移模式控制 (WASDQE):");
        puts("  W/S - 前后移动");
        puts("  A/D - 左右移动");
        puts("  Q/E - 升降移动");
        puts("旋转模式控制 (WASDQE):");
        puts("  W/S - 俯仰角(Pitch)");
        puts("  A/D - 偏航角(Yaw)");
        puts("  Q/E - 翻滚角(Roll)");
        puts("其他功能:");
        puts("  R - 世界坐标系");
        puts("  F - 图像坐标系");
        puts("  Z - 开车位置");
        puts("  X - 取矿位置");
        puts("  C - 兑矿位置");
        puts("  J - 气泵开启");
        puts("  K - 气泵关闭");
        puts("  SPACE - 退出程序");
    }

    void process_key(char c)
    {
        auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
        bool publish_twist = false;
        std_msgs::msg::Bool cmd_msg;

        switch (c)
        {
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
            if (current_mode_ == TRANSLATION)
            {
                twist_msg->twist.linear.x = 1.0 * vel_cmd_;
            }
            else
            {
                twist_msg->twist.angular.y = 1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_S:
            if (current_mode_ == TRANSLATION)
            {
                twist_msg->twist.linear.x = -1.0 * vel_cmd_;
            }
            else
            {
                twist_msg->twist.angular.y = -1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_A:
            if (current_mode_ == TRANSLATION)
            {
                twist_msg->twist.linear.y = -1.0 * vel_cmd_;
            }
            else
            {
                twist_msg->twist.angular.z = 1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_D:
            if (current_mode_ == TRANSLATION)
            {
                twist_msg->twist.linear.y = 1.0 * vel_cmd_;
            }
            else
            {
                twist_msg->twist.angular.z = -1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_Q:
            if (current_mode_ == TRANSLATION)
            {
                twist_msg->twist.linear.z = 1.0 * vel_cmd_;
            }
            else
            {
                twist_msg->twist.angular.x = 1.0 * vel_cmd_;
            }
            publish_twist = true;
            break;
        case KEYCODE_E:
            if (current_mode_ == TRANSLATION)
            {
                twist_msg->twist.linear.z = -1.0 * vel_cmd_;
            }
            else
            {
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

        // 退出程序
        case KEYCODE_SPACE:
            RCLCPP_INFO(get_logger(), "退出程序");
            rclcpp::shutdown();
            break;
        // 气泵开启
        case KEYCODE_J:
            cmd_msg.data = true;
            cmd_pub_->publish(cmd_msg);
            RCLCPP_INFO(get_logger(), "气泵开启");
            break;
        // 气泵关闭
        case KEYCODE_K:
            cmd_msg.data = false;
            cmd_pub_->publish(cmd_msg);
            RCLCPP_INFO(get_logger(), "气泵关闭");
            break;
        }

        if (publish_twist)
        {
            twist_msg->header.stamp = now();
            twist_msg->header.frame_id = frame_to_publish_;
            twist_pub_->publish(std::move(twist_msg));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardServo>();
    node->keyLoop();
    rclcpp::shutdown();
    return 0;
}