#include "franka_keyboard_control/serial_driver.hpp"

namespace ext_serial_driver
{
    Port::Port(int thread_num) : owned_ctx_{new IoContext(thread_num)}, serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
    {
        // RCLCPP_INFO(get_logger(), "Port Initialized!");
        printf("[INFO] Port Initialized!\n");
    }
    Port::~Port()
    {
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }

        if (serial_driver_->port()->is_open())
        {
            serial_driver_->port()->close();
        }

        if (owned_ctx_)
        {
            owned_ctx_->waitForExit();
        }
    }
    void Port::getParams(std::string device_name, uint32_t baud_rate, std::string fc_string, std::string pt_string, std::string sb_string)
    {
        using FlowControl = drivers::serial_driver::FlowControl;
        using Parity = drivers::serial_driver::Parity;
        using StopBits = drivers::serial_driver::StopBits;

        uint32_t baud_rate_{};
        auto fc = FlowControl::NONE;
        auto pt = Parity::NONE;
        auto sb = StopBits::ONE;

        device_name_ = device_name;
        baud_rate_ = baud_rate;
        if (fc_string == "none")
        {
            fc = FlowControl::NONE;
        }
        else if (fc_string == "hardware")
        {
            fc = FlowControl::HARDWARE;
        }
        else if (fc_string == "software")
        {
            fc = FlowControl::SOFTWARE;
        }
        else
        {
            throw std::invalid_argument{
                "The flow_control parameter must be one of: none, software, or hardware."};
        }

        if (pt_string == "none")
        {
            pt = Parity::NONE;
        }
        else if (pt_string == "odd")
        {
            pt = Parity::ODD;
        }
        else if (pt_string == "even")
        {
            pt = Parity::EVEN;
        }
        else
        {
            throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
        }

        if (sb_string == "1" || sb_string == "1.0")
        {
            sb = StopBits::ONE;
        }
        else if (sb_string == "1.5")
        {
            sb = StopBits::ONE_POINT_FIVE;
        }
        else if (sb_string == "2" || sb_string == "2.0")
        {
            sb = StopBits::TWO;
        }
        else
        {
            throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
        }

        device_config_ =
            std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate_, fc, pt, sb);
    }
    void Port::reopenPort()
    {
        perror("Attempting to reopen port");
        try
        {
            if (serial_driver_->port()->is_open())
            {
                serial_driver_->port()->close();
            }
            serial_driver_->port()->open();
            perror("Successfully reopened port");
        }
        catch (const std::exception &ex)
        {
            fprintf(stderr, "Error while reopening port: %s\n", ex.what());
            if (rclcpp::ok())
            {
                rclcpp::sleep_for(std::chrono::seconds(1));
                reopenPort();
            }
        }
    }
}