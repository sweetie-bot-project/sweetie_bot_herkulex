#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>
#include <rtt/RTT.hpp>

#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexPacket.h"
#include "orocos/sensor_msgs/typekit/JointState.h"

class HerkulexArray : public RTT::TaskContext
{

	// COMPONENT INTERFACE
	protected:
		// PROPERTIES
		RTT::PropertyBag servos_prop;
		unsigned int tryout_prop;
		double timeout;

	protected:
		// PORTS
		RTT::OutputPort<sensor_msgs::JointState> joints_port;
		// OPERATIONS: PROVIDED (low level interface)
		bool cmdAckCP(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt);
		// OPERATIONS: REQUIRED (low level interface)
		RTT::OperationCaller<void(const herkulex_msgs::HerkulexPacket& pkt)> cmdReqCP;
		// OPERATIONS: PROVIDED (high level interface)
		std::vector<std::string> listRegister(const std::string& servo)
		unsigned long getRegister(const std::string& servo, const std::string& reg);
		bool setRegister(const std::string& servo, const std::string& reg, unsigned int val);
		void printRegisterList(const std::string& servo);
		void printRegister(const std::string& servo, const std::string& reg);
		void printAllRegisters(const std::string& servo);
		unsigned int getStatus(const std::string& servo);
		bool clearStatus(const std::string& servo);

	protected:
		// Syncronous package exchange with servo array.
		bool sendRequest(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket&, sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& ack);

	public:
		HerkulexArray(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
#endif
