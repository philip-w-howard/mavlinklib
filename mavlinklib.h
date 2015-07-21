// mavlinklib.h

#pragma once

#include <iostream>
#include "mavlink/ardupilotmega/mavlink.h"

using namespace System;

namespace mavlinklib {

	public ref class MavlinkProcessor
	{
	public:
		static const int BUFFER_SIZE = MAVLINK_MAX_PACKET_LEN;

		static void check_init()
		{
			if (mav_msg == nullptr)
			{
				mav_msg = new mavlink_message_t[2]();
			}

			if (mav_status == nullptr)
			{
				mav_status = new mavlink_status_t[2]();
			}
		}

		static array<uint8_t>^ create_heartbeat(uint8_t sysid, uint8_t compid)
		{
			array<uint8_t>^ buff;
			uint8_t t_buff[BUFFER_SIZE];
			int len;

			uint8_t system_type = MAV_TYPE_GCS;
			uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

			uint8_t system_mode = 0;
			uint32_t custom_mode = 0;
			uint8_t system_state = 0;

			// Initialize the required buffers
			mavlink_message_t msg;

			// Pack the message
			mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
			len = mavlink_msg_to_send_buffer(t_buff, &msg);

			buff = gcnew array< uint8_t >(len);

			for (int ii = 0; ii < len; ii++)
			{
				buff[ii] = t_buff[ii];
			}

			return buff;
		}

		static array<uint8_t>^ process_byte(int channel, uint8_t byte)
		{
			array<uint8_t>^ buff;

			check_init();

			if (mavlink_parse_char(channel, byte, &mav_msg[channel], &mav_status[channel]))
			{
				uint8_t t_buff[BUFFER_SIZE];
				int len;
				len = mavlink_msg_to_send_buffer(t_buff, &mav_msg[channel]);
				buff = gcnew array< uint8_t >(len);
				for (int ii = 0; ii < len; ii++)
				{
					buff[ii] = t_buff[ii];
				}
				if (mav_status[channel].msg_received != MAVLINK_FRAMING_OK)
				{
					std::cerr << "Framing error in returned message" << std::endl;
				}
			}
			else {
				buff = gcnew array<uint8_t>(0);
			}

			return buff;
		}
	protected:
		static mavlink_message_t *mav_msg = nullptr;
		static mavlink_status_t  *mav_status = nullptr;

	};
}
