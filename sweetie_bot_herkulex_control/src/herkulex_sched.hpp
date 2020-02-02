#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexPacket.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexState.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexJointState.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexSchedStatistics.h>
#include <sweetie_bot_herkulex_msgs/typekit/ServoGoal.h>

#include "herkulex_servo.hpp"

#define SCHED_STATISTICS

namespace herkulex 
{

class HerkulexSched : public RTT::TaskContext
{

	protected:
		enum SchedulerState {
			SEND_JOG, SEND_JOG_WAIT, SEND_READ_REQ, RECEIVE_READ_ACK, CM_ROUND
		};

		typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;
		typedef sweetie_bot_herkulex_msgs::HerkulexState HerkulexState;
		typedef sweetie_bot_herkulex_msgs::HerkulexJointState HerkulexJointState;
		typedef sweetie_bot_herkulex_msgs::HerkulexSchedStatistics HerkulexSchedStatistics;
		typedef sweetie_bot_herkulex_msgs::ServoGoal ServoGoal;

		class SchedTimer : public RTT::os::Timer {
				HerkulexSched * owner;
			public:
				SchedTimer(HerkulexSched * _owner) : Timer(2, ORO_SCHED_RT, RTT::os::HighestPriority), owner(_owner) {}
				void timeout(TimerId id) {
					owner->trigger();
				}
		};
		enum SchedTimers {
			REQUEST_TIMEOUT_TIMER = 0,
			ROUND_TIMER = 1,
		};

	protected:
		// logger
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
		// scheduler state
		SchedulerState sched_state;
		int poll_index;
		int poll_end_index;
		// package buffers
		RTT::base::BufferLockFree<HerkulexPacket> cm_req_buffer;
		RTT::base::BufferLockFree<HerkulexPacket> ack_buffer;
		HerkulexPacket req_pkt;
		// data port buffers
		sensor_msgs::JointState joints;
		HerkulexJointState states;
		ServoGoal goals;
		// timer
		SchedTimer timer;
#ifdef SCHED_STATISTICS
		HerkulexSchedStatistics statistics;
		const RTT::os::TimeService * time_service;
		RTT::os::TimeService::ticks statistics_sync_timestamp;
#endif /* SCHED_STATISTICS */

	// COMPONENT INTERFACE
	protected:
		// Ports
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::InputPort<ServoGoal> goals_port;
		RTT::OutputPort<sensor_msgs::JointState> joints_port;
		RTT::OutputPort<HerkulexJointState> states_port;
#ifdef SCHED_STATISTICS
		RTT::OutputPort<HerkulexSchedStatistics> statistics_port;
#endif /* SCHED_STATISTICS */

		// Properties
		double period_RT_JOG;
		double period_RT_read;
		double period_CM;
		bool detailed_state;
		std::vector<std::string> poll_list;
		int poll_round_size;
		double timeout;

	protected:
		// helper funcions
		void clearPortBuffers();

	protected:
		// OPERATIONS: DATA LINK INTERFACE
		// Operations: provided
		void receivePacketDL(const HerkulexPacket& pkt);
		// Operations: required
		RTT::OperationCaller<void(const HerkulexPacket& pkt)> sendPacketDL;
		RTT::SendHandle<void(const HerkulexPacket& pkt)> sendPacketDL_handle;
		RTT::OperationCaller<void()> waitSendPacketDL;
		// OPERATIONS: CONFIGURATION AND MONITORING INTERFACE
		// Operations: provided
		void sendPacketCM(const HerkulexPacket& pkt);
		// Operations: required
		RTT::OperationCaller<void(const HerkulexPacket& pkt)> receivePacketCM;
		// OPERATIONS: PROTOCOL
		RTT::OperationCaller<bool (HerkulexPacket& req, const ServoGoal& goal)> reqIJOG;
		RTT::OperationCaller<bool (HerkulexPacket& req, const std::string& servo)> reqStatus;
		RTT::OperationCaller<bool (const HerkulexPacket& ack, const std::string& servo, double& temperature, servo::Status& status)> ackStatus;
		RTT::OperationCaller<bool (HerkulexPacket& req, const std::string& servo)> reqStatusExtended;
		RTT::OperationCaller<bool (const HerkulexPacket& ack, const std::string& servo, HerkulexState& state, servo::Status& status)> ackStatusExtended;
		RTT::OperationCaller<bool (HerkulexPacket& req, const std::string& servo)> reqPosVel;
		RTT::OperationCaller<bool (const HerkulexPacket& ack, const std::string& servo, double& pos, double& vel, servo::Status& status)> ackPosVel;
		RTT::OperationCaller<bool (HerkulexPacket& req, const std::string& servo)> reqPosVelExtended;
		RTT::OperationCaller<bool (const HerkulexPacket& ack, const std::string& servo, HerkulexJointState& state, servo::Status& status)> ackPosVelExtended;

	public:
		HerkulexSched(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}

#endif
