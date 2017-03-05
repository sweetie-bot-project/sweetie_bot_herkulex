#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>
#include <rtt/RTT.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include "orocos/sweetie_bot_herkulex_msgs/typekit/HerkulexPacket.h"

namespace herkulex
{

class HerkulexDriver : public RTT::TaskContext
{
	protected: 
		typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;

		enum ReceiverState {
			HEADER1, HEADER2, PACKET_SIZE, SERVO_ID, CMD, CHECKSUM1, CHECKSUM2, DATA 
		};
		//static const unsigned int BUFFER_SIZE;
		//static const unsigned int HEADER_SIZE;

	protected:
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
		// Port file handler
		int port_fd;
		// Receiver state
		ReceiverState recv_state;
		// Receive buffers
		HerkulexPacket recv_pkt;
		unsigned char recv_pkt_size;
		unsigned char recv_pkt_checksum1;

	    // COMPONENT INTERFACE
	protected:
		// Properties
		std::string port_name_prop;
		unsigned int baudrate_prop;

	protected:
		// Operations: provided
		void sendPacketDL(const HerkulexPacket& pkt);
		void waitSendPacketDL();
		// Operations: required
		RTT::OperationCaller<void(const HerkulexPacket& pkt)> receivePacketDL;

	public:
		HerkulexDriver(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}
#endif
