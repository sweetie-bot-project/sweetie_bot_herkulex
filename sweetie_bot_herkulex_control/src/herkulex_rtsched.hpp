#ifndef OROCOS_HERKULEX_RTSCHED_COMPONENT_HPP
#define OROCOS_HERKULEX_RTSCHED_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <sweetie_bot_logger/logger.hpp>

#include <sensor_msgs/typekit/JointState.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexPacket.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexState.h>
#include <sweetie_bot_herkulex_msgs/typekit/HerkulexSchedStatistics.h>

#include "herkulex_servo.hpp"

#define SCHED_STATISTICS

namespace herkulex 
{

class HerkulexRTSched : public RTT::TaskContext
{

	protected:
		enum SchedulerState {
			RT_ROUND_REQ, RT_ROUND_ACK, CM_ROUND
		};

		typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;
		typedef sweetie_bot_herkulex_msgs::HerkulexState HerkulexState;
		typedef sweetie_bot_herkulex_msgs::HerkulexSchedStatistics HerkulexSchedStatistics;
		typedef sensor_msgs::JointState JointState;

		class SchedTimer : public RTT::os::Timer {
				HerkulexRTSched * owner;
			public:
				SchedTimer(HerkulexRTSched * _owner) : Timer(3, ORO_SCHED_RT, RTT::os::HighestPriority), owner(_owner) {}
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
		// package buffers
		RTT::base::BufferLockFree<HerkulexPacket> cm_req_buffer;
		RTT::base::BufferLockFree<HerkulexPacket> ack_buffer;
		HerkulexPacket req_pkt;
		// data port buffers
		sensor_msgs::JointState joints_ref;
		sensor_msgs::JointState joints;
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
		RTT::InputPort<JointState> joints_ref_port;
		RTT::OutputPort<JointState> joints_port;
#ifdef SCHED_STATISTICS
		RTT::OutputPort<HerkulexSchedStatistics> statistics_port;
#endif /* SCHED_STATISTICS */

		// Properties
		double period_RT;
		double period_CM;
		double req_timeout;

	protected:
		// helper funcions
		void clearPortBuffers();
		void checkRTAckPackages();
		void forwardAckPackagesToCM();

	protected:
		// OPERATIONS: DATA LINK INTERFACE
		// Operations: provided
		void receivePacketDL(const HerkulexPacket& pkt);
		// Operations: required
		RTT::OperationCaller<void(const HerkulexPacket& pkt)> sendPacketDL;
		RTT::OperationCaller<void()> waitSendPacketDL;
		// OPERATIONS: CONFIGURATION AND MONITORING INTERFACE
		// Operations: provided
		void sendPacketCM(const HerkulexPacket& pkt);
		// Operations: required
		RTT::OperationCaller<void(const HerkulexPacket& pkt)> receivePacketCM;
		// OPERATIONS: PROTOCOL
		RTT::OperationCaller<bool (HerkulexPacket& req, const JointState& cmd)> reqRT_EXCHANGE;
		RTT::OperationCaller<bool (const HerkulexPacket& ack, JointState& state, double& temperature, servo::Status& status)> ackRT_EXCHANGE;

	public:
		HerkulexRTSched(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};

}

#endif
