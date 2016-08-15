#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>
#include <rtt/RTT.hpp>

#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexPacket.h"
#include "orocos/sweetie_bot_hardware_herkulex/herkulex_servo.hpp"
#include "orocos/sweetie_bot_hardware_herkulex/herkulex_servo_drs101.hpp"
#include "orocos/sensor_msgs/typekit/JointState.h"

class HerkulexArray : public RTT::TaskContext
{
	protected:
		static const unsigned long READ_ERROR;

	protected:
		// cached servo configuration
		herkulex_servo::HerkulexServoArray servos; /**< Servo sructures */
		std::map< std::string, std::shared_ptr<RegisterValues> > servos_init; /**< Registers initializeted on startup. */
		std::shared_ptr<herkulex_servo::HerkulexServo> broadcast; /**< Fictive servo with broadcast ID */
		// port buffer
		sensor_msgs::JointState joints;
		// packet buffers
		RTT::base::BufferLockFree< sweetie_bot_hardware_herkulex_msgs::HerkulexPacket > ack_buffer; /**< Buffer for received ACK packets. */ 
		RTT::os::Mutex ack_mutex;
		RTT::os::Condition ack_cond;
		RTT::os::Timer timeout_timer;
		// internals
		bool break_loop_flag;

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
		void receivePacketCM(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt);
		// OPERATIONS: REQUIRED (low level interface)
		RTT::OperationCaller<void(const herkulex_msgs::HerkulexPacket& pkt)> sendPacketCM;

		// OPERATIONS: PROVIDED (high level interface)
		std::vector<std::string> listServos();
		std::vector<std::string> listRegisters(const std::string& servo);

		unsigned long getRegisterRAM(const std::string& servo, const std::string& reg);
		bool setRegisterRAM(const std::string& servo, const std::string& reg, unsigned int val);

		unsigned int getStatus(const std::string& servo);
		bool clearStatus(const std::string& servo);

		bool resetServo(const std::string& servo);
		bool resetAllServos();

		bool getJointStates();

		void printServoStatus(const std::string& servo);
		void printAllServoStatuses();
		void printErrorServoStatuses();
		void printRegisterRAM(const std::string& servo, const std::string& reg);
		void printAllRegistersRAM(const std::string& servo);

		// OPERATIONS: PROVIDED (protocol interface)
		bool reqIJOG(HerkulexPacket& req, const sweetie_bot_core_msgs::ServoGoal& goal);
		bool reqPosVel(HerkulexPacket& req, const std::string servo);
		bool ackPosVel(HerkulexPacket& ack, const std::string servo, double& pos, double& vel);

	protected:
		// Syncronous package exchange with servo array.
		bool cmdSync(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket&, sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& ack);
		// Helper fuctions
		std::string statusToString(Status status) ;

	public:
		HerkulexArray(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
#endif
