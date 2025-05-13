#ifndef EXT_SERIAL_DRIVER__PACKET_HPP_
#define EXT_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace ext_serial_driver
{
	// up packet
	struct UpReceivePacket
	{
		uint8_t header = 0xA8;

		uint8_t key_code;

		uint16_t checksum = 0;
	} __attribute__((packed));

	struct SendPacket
	{
		uint8_t header = 0xB8;

		float joint0_state;
		float joint1_state;
		float joint2_state;
		float joint3_state;
		float joint4_state;
		float joint5_state;
		float joint6_state;

		uint16_t checksum = 0;
	} __attribute__((packed));

	/********************************************************/
	/* template                                             */
	/********************************************************/

	template <typename T>
	inline T fromVector(const std::vector<uint8_t> &data)
	{
		T packet;
		std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
		return packet;
	}

	template <typename T>
	inline std::vector<uint8_t> toVector(const T &data)
	{
		std::vector<uint8_t> packet(sizeof(T));
		std::copy(
			reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
			packet.begin());
		return packet;
	}

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
