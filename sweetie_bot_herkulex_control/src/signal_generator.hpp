#ifndef OROCOS_SIGNAL_GENERATOR_COMPONENT_HPP
#define OROCOS_SIGNAL_GENERATOR_COMPONENT_HPP

#include <string>
#include <rtt/RTT.hpp>

#include <sweetie_bot_logger/logger.hpp>
#include <sensor_msgs/typekit/JointState.h>

namespace herkulex {

class SignalGenerator : public RTT::TaskContext
{
	protected: 
		typedef sensor_msgs::JointState JointState;

	protected:
		// logger
#ifdef SWEETIEBOT_LOGGER
		sweetie_bot::logger::SWEETIEBOT_LOGGER log;
#else
		sweetie_bot::logger::LoggerRTT log;
#endif
		// COMPONENT STATE
		double t; // current time
		JointState joints;

	protected:
		// COMPONENT INTERFACE
		// Properties
		std::vector<std::string> joint_names_prop;
		std::vector<std::string> signals_prop;
		std::vector<double> freqs_prop;
		std::vector<double> amplitudes_prop;
		std::vector<double> phases_prop;
		std::vector<double> masses_prop;
		// Ports
		RTT::OutputPort<JointState> joints_port;
	
	public:
		SignalGenerator(const std::string& name);
		bool startHook();
		void updateHook();
		void stopHook();
};

}

#endif
