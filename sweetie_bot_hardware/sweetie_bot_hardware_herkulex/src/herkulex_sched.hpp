#ifndef OROCOS_HERKULEX_DRIVER_COMPONENT_HPP
#define OROCOS_HERKULEX_DRIVER_COMPONENT_HPP

#include <string>

#include <rtt/RTT.hpp>
#include <rtt/os/Timer.hpp>

#include <orocos/sensor_msgs/typekit/JointState.h>
#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexPacket.h"
#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexServoState.h"
#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/HerkulexStatistics.h"
#include "orocos/sweetie_bot_hardware_herkulex_msgs/typekit/ServoGoal.h"

#define SCHED_STATISTICS

class HerkulexSched : public RTT::TaskContext
{

	protected:
		enum SchedulerState {
			SEND_JOG, SEND_JOG_WAIT, SEND_READ_REQ, RECEIVE_READ_ACK, CM_ROUND
		};

		typedef sweetie_bot_hardware_herkulex_msgs::HerkulexPacket HerkulexPacket;
		typedef sweetie_bot_hardware_herkulex_msgs::HerkulexServoState HerkulexServoState;
		typedef sweetie_bot_hardware_herkulex_msgs::HerkulexStatistics HerkulexStatistics;
		typedef sweetie_bot_hardware_herkulex_msgs::ServoGoal ServoGoal;

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
		SchedulerState sched_state;
		int poll_list_index;
		// package buffers
		RTT::base::BufferLockFree<HerkulexPacket> cm_req_buffer;
		RTT::base::BufferLockFree<HerkulexPacket> ack_buffer;
		HerkulexPacket req_pkt;
		// data port buffers
		sensor_msgs::JointState joints;
		HerkulexServoState states;
		ServoGoal goals;
		// timer
		SchedTimer timer;
#ifdef SCHED_STATISTICS
		HerkulexStatistics statistics;
		const RTT::os::TimeService * time_service;
		RTT::os::TimeService::ticks statistics_round_timestamp;
#endif /* SCHED_STATISTICS */

	    // COMPONENT INTERFACE
	protected:
		// Ports
		RTT::InputPort<RTT::os::Timer::TimerId> sync_port;
		RTT::InputPort<ServoGoal> goals_port;
		RTT::OutputPort<sensor_msgs::JointState> joints_port;
		RTT::OutputPort<HerkulexServoState> states_port;
#ifdef SCHED_STATISTICS
		RTT::OutputPort<HerkulexStatistics> statistics_port;
#endif /* SCHED_STATISTICS */

		// Properties
		double period_RT_JOG;
		double period_RT_read;
		double period_CM;
		bool detailed_state;
		std::vector<std::string> poll_list;
		double timeout;

	protected:
		// helper funcions
		void clearPortBuffers();

	protected:
		// OPERATIONS: DATA LINK INTERFACE
		// Operations: provided
		void receivePacketDL(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt);
		// Operations: required
		RTT::OperationCaller<void(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt)> sendPacketDL;
		// OPERATIONS: CONFIGURATION AND MONITORING INTERFACE
		// Operations: provided
		void sendPacketCM(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt);
		// Operations: required
		RTT::OperationCaller<void(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt)> receivePacketCM;
		// OPERATIONS: PROTOCOL
		RTT::OperationCaller<bool (sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& req, const sweetie_bot_hardware_herkulex_msgs::ServoGoal& goal)> reqIJOG;
		RTT::OperationCaller<bool (sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& req, const std::string& servo)> reqPosVel;
		RTT::OperationCaller<bool (const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& ack, const std::string& servo, double& pos, double& vel, unsigned int& status)> ackPosVel;
		RTT::OperationCaller<bool (sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& req, const std::string& servo)> reqState;
		RTT::OperationCaller<bool (const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& ack, const std::string& servo, sweetie_bot_hardware_herkulex_msgs::HerkulexServoState& state, unsigned int& status)> ackState;

	public:
		HerkulexSched(std::string const& name);
		bool configureHook();
		bool startHook();
		void updateHook();
		void stopHook();
		void cleanupHook();
};
#endif
