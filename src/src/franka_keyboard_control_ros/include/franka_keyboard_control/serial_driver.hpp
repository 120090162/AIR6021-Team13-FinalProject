#ifndef RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER_HPP_

// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <serial_driver/serial_driver.hpp>

namespace ext_serial_driver
{
	class Port
	{
	public:
		Port(int thread_num);
		~Port();

		// port function
		void getParams(std::string device_name, uint32_t baud_rate, std::string fc_string, std::string pt_string, std::string sb_string);
		void reopenPort();

		// Serial port
		std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
		std::string device_name_;
		std::unique_ptr<IoContext> owned_ctx_; // IoContext 定义thread个数
		std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

		std::thread receive_thread_;

		// private:
		// 	std::unique_ptr<IoContext> owned_ctx_; // IoContext 定义thread个数
	};
}

#endif // RM_SERIAL_DRIVER_HPP_