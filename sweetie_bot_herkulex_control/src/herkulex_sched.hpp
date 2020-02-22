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
			SEND_JOG, SEND_JOG_WAIT, SEND_READ_REQ, WAIT_READ_ACK, CM_ROUND
		};

		typedef sweetie_bot_herkulex_msgs::HerkulexPacket HerkulexPacket;
		typedef sweetie_bot_herkulex_msgs::HerkulexState HerkulexState;
		typedef sweetie_bot_herkulex_msgs::HerkulexJointState HerkulexJointState;
		typedef sweetie_bot_herkulex_msgs::HerkulexSchedStatistics HerkulexSchedStatistics;
		typedef sweetie_bot_herkulex_msgs::ServoGoal ServoGoal;

		class SchedTimer : public RTT::os::Timer {
				HerkulexSched * owner;
			public:
				SchedTimer(HerkulexSched * _owner) : Timer(3, ORO_SCHED_RT, RTT::os::HighestPriority), owner(_owner) {}
				void timeout(TimerId id) {
					owner->trigger();
				}
		};
		enum SchedTimers {
			REQUEST_PERIOD_TIMER = 0,
			REQUEST_TIMEOUT_TIMER = 1,
			ROUND_TIMER = 2,
		};

		template <typename T, size_t N> class Queue
		{
			public:
				typedef typename std::array<T, N>::iterator iterator;
				typedef typename std::array<T, N>::const_iterator const_iterator;
			private:
				std::array<T, N> values;
				iterator tail;
			public:
				Queue() { tail = values.begin(); }

				iterator enqueue(const T& value) {
					if (tail == values.end()) dequeue();
					*tail = value;
					return tail++;
				}
				bool dequeue() {
					if (tail == values.begin()) return false;
					for(iterator it = values.begin(); it + 1 != tail; it++)
						*it = *(it + 1);
					tail--;
					return true;
				}
				bool dequeue(T& value) {
					if (tail == values.begin()) return false;
					value = values.front();
					return dequeue();
				}
				bool dequeue(iterator it) {
					if (it == tail) return false;
					it++;
					iterator dst = begin();
					while (it != tail) *(dst++) = *(it++); 
					tail = dst;
					return true;
				}
				void clear() { tail = values.begin(); }
				bool empty() { return tail == values.begin(); }
				bool full() { return tail == values.end(); }
				size_t size() { return tail - values.begin(); }
				size_t capacity() { return N; }
				iterator begin() { return values.begin(); }
				iterator end() { return tail; }
				const_iterator begin() const { return values.begin(); }
				const_iterator end() const { return tail; }
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
		Queue<int, 2> poll_index_ack_queue;
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
		double req_timeout;
		double req_period;
		bool detailed_state;
		std::vector<std::string> poll_list;
		int poll_round_size;

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
		RTT::OperationCaller<bool (HerkulexPacket& req, const ServoGoal& goal)> reqIJOG;
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
