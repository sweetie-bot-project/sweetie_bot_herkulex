#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>
#include <map>
#include <unordered_map>
#include <memory>

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <orocos/sensor_msgs/typekit/JointState.h>
#include <orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexPacket.h>
//#include <orocos/sweetie_bot_core_msgs/typekit/ServoGoal.h>

#include "herkulex_servo.hpp"
#include "herkulex_servo_drs101.hpp"

class HerkulexArray : public RTT::TaskContext
{
	protected:
		class TimeoutTimer : public RTT::os::Timer 
		{
				HerkulexArray * owner;
			public:
				TimeoutTimer(HerkulexArray * _owner) : Timer(1, ORO_SCHED_RT, RTT::os::HighestPriority), owner(_owner) {}
				void timeout(TimerId)
			   	{
					owner->ack_mutex.lock();
					owner->ack_cond.broadcast();
					owner->ack_mutex.unlock();
				}
		};

	protected:
		// constants
		static const unsigned long READ_ERROR;
		static const unsigned int JOG_STOP;
		static const unsigned int JOG_POSITION;
		static const unsigned int JOG_SPEED;

	protected:
		// cached servo configuration
		herkulex_servo::HerkulexServoArray servos; /**< Servo sructures */
		std::map< std::string, std::shared_ptr<herkulex_servo::RegisterValues> > servos_init; /**< Registers initializeted on startup. */
		std::shared_ptr<herkulex_servo::HerkulexServo> broadcast; /**< Fictive servo with broadcast ID */
		// port buffer
		sensor_msgs::JointState joints;
		// packet buffers
		herkulex_servo::HerkulexPacket req_pkt;
		RTT::base::BufferLockFree< herkulex_servo::HerkulexPacket > ack_buffer; /**< Buffer for received ACK packets. */ 
		RTT::os::Mutex ack_mutex;
		RTT::os::Condition ack_cond;
		TimeoutTimer timeout_timer;
		const TimeoutTimer::TimerId TIMEOUT_TIMER_ID = 0;
		// internals
		bool break_loop_flag;

	// COMPONENT INTERFACE
	protected:
		// PROPERTIES
		RTT::PropertyBag servos_prop;
		unsigned int tryouts_prop;
		double timeout_prop;

	protected:
		// PORTS
		RTT::OutputPort<sensor_msgs::JointState> joints_port;
		// OPERATIONS: PROVIDED (low level interface)
		void receivePacketCM(const herkulex_servo::HerkulexPacket& pkt);
		// OPERATIONS: REQUIRED (low level interface)
		RTT::OperationCaller<void(const herkulex_servo::HerkulexPacket& pkt)> sendPacketCM;

		// OPERATIONS: PROVIDED (high level interface)
		std::vector<std::string> listServos();
		std::vector<std::string> listServoRegistersRAM(const std::string& servo);

		unsigned long getRegisterRAM(const std::string& servo, const std::string& reg);
		bool setRegisterRAM(const std::string& servo, const std::string& reg, unsigned int val);

		bool setGoalRaw(const std::string& servo, unsigned int mode, unsigned int goal, unsigned int playtime);
		bool setGoal(const std::string& servo, unsigned int mode, double goal, double playtime);

		unsigned long getStatus(const std::string& servo);
		bool clearStatus(const std::string& servo);

		bool resetServo(const std::string& servo);
		bool resetAllServos();

		bool publishJointStates();

		void printServoStatus(const std::string& servo);
		void printAllServoStatuses();
		void printErrorServoStatuses();
		void printRegisterRAM(const std::string& servo, const std::string& reg);
		void printAllRegistersRAM(const std::string& servo);

		// OPERATIONS: PROVIDED (protocol interface)
		//bool reqIJOG(HerkulexPacket& req, const sweetie_bot_core_msgs::ServoGoal& goal);
		bool reqPosVel(herkulex_servo::HerkulexPacket& req, const std::string servo);
		bool ackPosVel(herkulex_servo::HerkulexPacket& ack, const std::string servo, double& pos, double& vel);

	protected:
		// interrupt waiting
		bool breakLoop();
		// Asyncronous and Syncronous package exchange with servo array.
		void sendPacket(const herkulex_servo::HerkulexPacket& req);
		bool sendRequest(const herkulex_servo::HerkulexPacket&, herkulex_servo::HerkulexServo::AckCallback);
		// Helper fuctions
		std::string statusToString(herkulex_servo::HerkulexServo::Status status) ;
		bool setServoRegisters(const herkulex_servo::HerkulexServo * s, const herkulex_servo::RegisterValues * reg_init);
		// servos access
		const herkulex_servo::HerkulexServo& getServo(const string& name); 
		bool addServo(std::shared_ptr<herkulex_servo::HerkulexServo> servo);
		/*bool checkServo(const string& name); 
		bool checkServoReg(const herkulex_servo::HerkulexServo& s, const string& reg);*/

	public:
		HerkulexArray(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
#endif
