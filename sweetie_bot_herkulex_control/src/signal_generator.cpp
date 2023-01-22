#include "signal_generator.hpp"

#include <cmath>
#include <tuple>

using namespace sweetie_bot;
using namespace RTT;

namespace herkulex {

SignalGenerator::SignalGenerator(const std::string& name) : 
	TaskContext(name, Stopped),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("HerkulexArray");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
		this->fatal();
		return;
	}

	// properties
	this->addProperty("joint_names", joint_names_prop) 
		.doc("Names of joints.");
	this->addProperty("signals", signals_prop) 
		.doc("Signal types for each joint: 'sine', 'step'.");
	this->addProperty("freqs", freqs_prop) 
		.doc("Signal frequencies for each joint.");
	this->addProperty("amplitudes", amplitudes_prop) 
		.doc("Signal amplitudes for each joint.");
	this->addProperty("phases", phases_prop) 
		.doc("Phase parameter for each joint.");
	this->addProperty("masses", masses_prop) 
		.doc("Mass or inertia parameter of each joint");

	// ports
	this->addPort("out_joints", joints_port)
		.doc("Output signal");
}

bool SignalGenerator::startHook()
{
	int size = joint_names_prop.size();
	// check properties
	if (signals_prop.size() != size || freqs_prop.size() != size || amplitudes_prop.size() != size || phases_prop.size() != size || masses_prop.size() != size) {
		log(ERROR) << "Properties joint_names, freqs, amplitudes, phases, masses must have the same size" << endlog();
		return false;
	}
	// check signal types
	for(const std::string& s : signals_prop) {
		if (s != "sine" && s != "step") {
			log(ERROR) << "Unknown signal " << s << endlog();
		}
	}
	// check period 
	if (getPeriod() == 0.0) {
		log(ERROR) << "SignalGenerator Activity period must be non-zero." << endlog();
		return false;
	}
	// prepear buffers
	joints.name = joint_names_prop;
	joints.position.assign(size, 0.0);
	joints.velocity.assign(size, 0.0);
	joints.effort.assign(size, 0.0);
	// reset state
	t = 0.0;

	log(INFO) << "SignalGenerator is started" << endlog(); 
	return true;
}

static std::tuple<double, double, double> sine(double t, double amplitude, double freq, double phase) 
{
	double wfreq = 2*M_PI * freq;
	double a = wfreq * t + 2*M_PI * phase;
	double pos = amplitude * std::sin(a);
	double vel = amplitude * wfreq * std::cos(a);
	double accel = - amplitude * wfreq * wfreq * std::sin(a);
	return {pos, vel, accel};
}

static std::tuple<double, double, double> step(double t, double amplitude, double freq, double phase) 
{
	if (std::fmod(freq * t, 1.0) < phase) {
		return {amplitude, 0.0, 0.0};
	}
	else {
		return {0.0, 0.0, 0.0};
	}
}

void SignalGenerator::updateHook()
{
	int n_joints = joints.name.size();

	for(int k = 0; k < n_joints; k++) {
		// generate signal
		if (signals_prop[k] == "sine") {
			std::tie(joints.position[k], joints.velocity[k], joints.effort[k]) = sine(t, amplitudes_prop[k], freqs_prop[k], phases_prop[k]);
		}
		else if (signals_prop[k] == "step") {
			std::tie(joints.position[k], joints.velocity[k], joints.effort[k]) = step(t, amplitudes_prop[k], freqs_prop[k], phases_prop[k]);
		}
		// convert acceleration to effort
		joints.effort[k] *= masses_prop[k];
	}
	// publish message
	joints_port.write(joints);
	// increase time
	t += this->getPeriod();
}

void SignalGenerator::stopHook() 
{
	log(INFO) << "SignalGenerator is stopped!" << endlog(); 
}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(SignalGenerator)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
extern "C" {

	RTT_EXPORT RTT::TaskContext* createComponent(std::string instance_name);

	RTT::TaskContext* createComponent(std::string instance_name)
	{ 
    	return new herkulex::SignalGenerator(instance_name);
  	}
  
	RTT_EXPORT std::string getComponentType();
  	std::string getComponentType()
  	{
    	return "herkulex::SignalGenerator";
  	}
} /* extern "C" */

//ORO_CREATE_COMPONENT(herkulex::SignalGenerator)
