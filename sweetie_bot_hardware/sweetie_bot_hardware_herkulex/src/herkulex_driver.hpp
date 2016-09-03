#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>
#include <rtt/RTT.hpp>

#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexPacket.h"

class HerkulexDriver : public RTT::TaskContext
{
	protected: 
		enum ReceiverState {
			HEADER1, HEADER2, PACKET_SIZE, SERVO_ID, CMD, CHECKSUM1, CHECKSUM2, DATA 
		};
		//static const unsigned int BUFFER_SIZE;
		//static const unsigned int HEADER_SIZE;

	protected:
		// Port file handler
		int port_fd;
		// Receiver state
		ReceiverState recv_state;
		// Receive buffers
		sweetie_bot_hardware_herkulex_msgs::HerkulexPacket recv_pkt;
		unsigned char recv_pkt_size;
		unsigned char recv_pkt_checksum1;

	    // COMPONENT INTERFACE
	protected:
		// Properties
		std::string port_name_prop;
		unsigned int baudrate_prop;

	protected:
		// Operations: provided
		void sendPacketDL(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt);
		void waitSendPacketDL();
		// Operations: required
		RTT::OperationCaller<void(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt)> receivePacketDL;

	public:
		HerkulexDriver(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
#endif
