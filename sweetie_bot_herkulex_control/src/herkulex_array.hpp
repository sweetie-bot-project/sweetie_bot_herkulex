#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexPacket.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexServoState.h>
#include <sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>

#include "herkulex_servo.hpp"
#include "herkulex_servo_drs101.hpp"

namespace herkulex
{

class HerkulexArray : public RTT::TaskContext
{
public:
  typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;
  typedef sweetie_bot_herkulex_msgs::HerkulexServoState HerkulexServoState;
  typedef sweetie_bot_herkulex_msgs::ServoGoal ServoGoal;

protected:
  class TimeoutTimer : public RTT::os::Timer
  {
    HerkulexArray* owner;

  public:
    ~TimeoutTimer() {}
    TimeoutTimer(HerkulexArray* _owner) : Timer(1, ORO_SCHED_RT, RTT::os::HighestPriority), owner(_owner) {}
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
  // logger
#ifdef SWEETIEBOT_LOGGER
  sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
  sweetie_bot::logger::LoggerRTT log;
#endif
		// cached servo configuration
		servo::HerkulexServoArray servos; /**< Servo sructures */
		std::map< std::string, std::shared_ptr<herkulex::servo::RegisterValues> > servos_init; /**< Registers initializeted on startup. */
		std::shared_ptr<herkulex::servo::HerkulexServo> broadcast; /**< Fictive servo with broadcast ID. */
		std::shared_ptr<herkulex::servo::RegisterValues> broadcast_init; /**< Global servo configuration. */
		// port buffer
		sensor_msgs::JointState joints;
		// packet buffers
		HerkulexPacket req_pkt;
		RTT::base::BufferLockFree< HerkulexPacket > ack_buffer; /**< Buffer for received ACK packets. */ 
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
		bool mass_reset_prop;
		double reset_delay_prop;

	protected:
		// PORTS
		RTT::OutputPort<sensor_msgs::JointState> joints_port;
		// OPERATIONS: PROVIDED (low level interface)
		void receivePacketCM(const HerkulexPacket& pkt);
		// OPERATIONS: REQUIRED (low level interface)
		RTT::OperationCaller<void(const HerkulexPacket& pkt)> sendPacketCM;

		// OPERATIONS: PROVIDED (high level interface)
		std::vector<std::string> listServos();
		std::vector<std::string> listServoRegistersRAM(const std::string& servo);

		unsigned int getRegisterRAM(const std::string& servo, const std::string& reg);
		bool setRegisterRAM(const std::string& servo, const std::string& reg, unsigned int val);

		unsigned int getRegisterEEP(const std::string& servo, const std::string& reg);
		bool setRegisterEEP(const std::string& servo, const std::string& reg, unsigned int val);

    bool setGoalRaw(const std::string& servo, unsigned char mode, unsigned int goal, unsigned int playtime);
    bool setGoal(const std::string& servo, unsigned char mode, double goal, double playtime);

		unsigned int getStatus(const std::string& servo);
		bool clearStatus(const std::string& servo);
		bool clearAllStatuses();

		bool resetServo(const std::string& servo);
		bool resetAllServos();
		bool setTorqueFree(const std::string& servo, bool torque_free);
		bool setAllServosTorqueFree(bool torque_on);

		bool publishJointStates();

		void printServoStatus(const std::string& servo);
		void printAllServoStatuses();
		void printErrorServoStatuses();
		void printRegisterRAM(const std::string& servo, const std::string& reg);
		void printAllRegistersRAM(const std::string& servo);

		void discoverServos();

		// OPERATIONS: PROVIDED (protocol interface)
		bool reqIJOG(HerkulexPacket& req, const ServoGoal& goal);
		bool reqPosVel(HerkulexPacket& req, const std::string& servo);
		bool ackPosVel(const HerkulexPacket& ack, const std::string& servo, double& pos, double& vel, servo::Status& status);
		bool reqState(HerkulexPacket& req, const std::string& servo);
		bool ackState(const HerkulexPacket& ack, const std::string& servo, HerkulexServoState& state, servo::Status& status);

	protected:
		// interrupt waiting
		bool breakLoop();
		// Asyncronous and Syncronous package exchange with servo array.
		void sendPacket(const HerkulexPacket& req);
		bool sendRequest(const HerkulexPacket&, servo::HerkulexServo::AckCallback);
		// Helper fuctions
		std::string statusToString(servo::Status status) ;
		bool setServoRegisters(const servo::HerkulexServo * s, const servo::RegisterValues * reg_init);
		bool setTorqueFree_impl(const servo::HerkulexServo * s, bool torque_free);
		// servos access
		const servo::HerkulexServo& getServo(const std::string& name); 
		bool addServo(std::shared_ptr<servo::HerkulexServo> servo);

	public:
		HerkulexArray(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

} // namespace herkulex
#endif
