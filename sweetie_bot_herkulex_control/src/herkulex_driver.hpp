#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>
#include <rtt/RTT.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sweetie_bot_herkulex_msgs/typekit/HerkulexPacket.h>

namespace herkulex
{

class HerkulexDriver : public RTT::TaskContext
{
	protected: 
		typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;

		enum ReceiverState {
			HEADER1, HEADER2, PACKET_SIZE, SERVO_ID, CMD, CHECKSUM1, CHECKSUM2, DATA, PARSE_ERROR
		};
		//static const unsigned int BUFFER_SIZE;
		//static const unsigned int HEADER_SIZE;

	protected:
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log, sender_log;
#else
		sweetie_bot::logger::LoggerRTT log, sender_log;
#endif
		// Port file handler
		int port_fd;
		// Receiver state
		ReceiverState recv_state;
		// Receive buffers
		unsigned char recv_buffer[HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE]; // raw receive buffer 
		ssize_t recv_buffer_size; // size of data in  raw recive buffer
		ssize_t recv_buffer_index; // points on first non processed byte in raw buffer
		ssize_t recv_buffer_header_index; // index of header first byte in recieve buffer
		HerkulexPacket recv_pkt;
		unsigned char recv_pkt_size;
		unsigned char recv_pkt_checksum1;
		// Sender state
		RTT::os::Mutex send_mutex; // mutex to prevent rances if sendPacketDL() is called from multiple threads.
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER send_log; // logger facilities for sendPacketDL() operation.
#else
		sweetie_bot::logger::LoggerRTT send_log; // logger facilities for sendPacketDL() operation.
#endif

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
	
	protected:
		bool gc(unsigned char& c) {
			if (recv_buffer_index >= recv_buffer_size) return false;
			c = recv_buffer[recv_buffer_index++];
			return true;
		}

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
