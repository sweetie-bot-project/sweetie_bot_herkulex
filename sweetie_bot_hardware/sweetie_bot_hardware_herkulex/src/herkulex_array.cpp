#include <iostream>
#include <stdexcept>

extern "C" {
#include <unistd.h>
}

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "herkulex_array.hpp"

#include "herkulex_servo.hpp"
#include "herkulex_servo_drs101.hpp"

using namespace RTT;
using namespace herkulex_servo;
using sweetie_bot::Logger;

namespace sweetie_bot 
{

std::ostream& operator<<(std::ostream& s, const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt) 
{
	std::ios::fmtflags fmt_flags( s.flags() );
	s << std::hex << std::setw(2) << std::setfill('0');
	s << "ACK packet: servo_id: " << (int) pkt.servo_id << " cmd: " << (int) pkt.command << " data(" << std::dec << pkt.data.size() << std::hex << "): ";
	for(auto c = pkt.data.begin(); c != pkt.data.end(); c++) log() << (int) *c << " ";
	s << std::endl;
	s.flags( fmt_flags );

	return s;
}


const unsigned long HerkulexArray::READ_ERROR = 0x10000;
const unsigned int HerkulexArray::JOG_STOP = herkulex_servo::JOGMode::STOP;
const unsigned int HerkulexArray::JOG_POSITION = herkulex_servo::JOGMode::POSITION_CONTROL;
const unsigned int HerkulexArray::JOG_SPEED = herkulex_servo::JOGMode::SPEED_CONTROL;

HerkulexArray::HerkulexArray(std::string const& name) : 
	TaskContext(name, PreOperational),
	sendPacketCM("sendPacketCM"),
	ack_buffer(10, HerkulexPacket(), true), // circular buffer for 10 packets
	timeout_timer(this),
	break_loop_flag(false),
	log("sweetie.core.herkulex.array")
{
	if (!log.ready()) {
		RTT::Logger::In in("HerkulexArray");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
		this->fatal();
		return;
	}

	// BASIC INITIALIZATION
	// Start timer thread.
	if (!timeout_timer.getActivity() || !timeout_timer.getActivity()->thread()) {
		log(ERROR) << "Unable to start timer thread.";
		this->fatal();
		return;
	}
	timeout_timer.getActivity()->thread()->start();
	// init brodcast object
	broadcast = std::shared_ptr<HerkulexServo>(new HerkulexServoDRS101("broadcast", 0xfe));
	broadcast_init = std::shared_ptr<RegisterValues>(new RegisterValues());

	// INTERFACE
	// CONSTANTS
	this->addConstant("READ_ERROR", READ_ERROR);
	this->addConstant("JOG_STOP", JOG_STOP);
	this->addConstant("JOG_POSITION", JOG_POSITION);
	this->addConstant("JOG_SPEED", JOG_SPEED);

	// PROPERTIES
	this->addProperty("servos", servos_prop) 
		.doc("Servo descriptions (PropertuBag). Format: \n"
			 "\t\t\t{\n"
			 "\t\t\t    PropertyBag servo_name1 { string servo_id, string servo_model, uint offset, bool reverse, PropertyBag register_init { uint register1, uint register2, ... } },\n"
			 "\t\t\t    PropertyBag servo_name2 { ... }\n"
			 "\t\t\t    ...\n"
			 "\t\t\t}");
	this->addProperty("tryouts", tryouts_prop)
		.doc("Number of attemts to perform one operation.")
		.set(3);
	this->addProperty("timeout", timeout_prop)
		.doc("Operation timeout (seconds).")
		.set(0.1);

	// PORTS
	this->addPort("out_joints", joints_port).doc("Publish JointState by request.");

	// OPERATIONS
	// Data link interface
	this->addOperation("receivePacketCM", &HerkulexArray::receivePacketCM, this, ClientThread) 
		.doc("Servo asyncronous responce hook operation.") 
		.arg("pkt", "Received HerkulexPacket.");
	this->requires()->addOperationCaller(sendPacketCM);
	
	// Application inteface.
	this->addOperation("listServos", &HerkulexArray::listServos, this, ClientThread)
		.doc("Return registered servo list.");
	this->addOperation("listServoRegistersRAM", &HerkulexArray::listServoRegistersRAM, this, ClientThread)
		.doc("Return servo register list.")
		.arg("servo", "Servo name.");

	this->addOperation("setRegisterRAM", &HerkulexArray::setRegisterRAM, this, OwnThread)
		.doc("Set servo register value. Return true on success.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.")
		.arg("val", "New register value (raw uint16).");
	this->addOperation("getRegisterRAM", &HerkulexArray::getRegisterRAM, this, OwnThread)
		.doc("Get servo register value. Return register raw value, on error return READ_ERROR.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.");

	this->addOperation("setRegisterEEP", &HerkulexArray::setRegisterEEP, this, OwnThread)
		.doc("Set servo register value. Return true on success.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.")
		.arg("val", "New register value (raw uint16).");
	this->addOperation("getRegisterEEP", &HerkulexArray::getRegisterEEP, this, OwnThread)
		.doc("Get servo register value. Return register raw value, on error return READ_ERROR.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.");

	this->addOperation("setGoalRaw", &HerkulexArray::setGoalRaw, this, OwnThread)
		.doc("Set servo goal. Return true on success.")
		.arg("servo", "Servo name.")
		.arg("mode", "JOG mode (raw uint8): JOG_STOP, JOG_POSITION, JOG_SPEED, LED_FLAGS.")
		.arg("goal", "New goal position or speed (raw uint16).")
		.arg("playtime", "Playtime (servo ticks).");
	this->addOperation("setGoal", &HerkulexArray::setGoal, this, OwnThread)
		.doc("Set servo goal. Return true on success.")
		.arg("servo", "Servo name.")
		.arg("mode", "JOG mode (raw uint8): JOG_STOP, JOG_POSITION, JOG_SPEED, LED_FLAGS.")
		.arg("goal", "New goal position (rad) or speed (rad/s).")
		.arg("playtime", "Playtime (s).");

	this->addOperation("getStatus", &HerkulexArray::getStatus, this, OwnThread)
		.doc("Get servo status. Return content of status registers (byte0=status_error, byte1=status_detail) or READ_ERROR.")
		.arg("servo", "Servo name.");
	this->addOperation("clearStatus", &HerkulexArray::clearStatus, this, OwnThread)
		.doc("Clear status register of servo. Return true on success.")
		.arg("servo", "Servo name.");
	this->addOperation("resetServo", &HerkulexArray::resetServo, this, OwnThread)
		.doc("Reset servo and init it again. Must clear all *hard* error status.")
		.arg("servo", "Servo name.");
	this->addOperation("resetAllServos", &HerkulexArray::resetAllServos, this, OwnThread)
		.doc("Reset all servos and init them again. Must clear all *hard* error status.");
	this->addOperation("setAllServosTorqueFree", &HerkulexArray::setAllServosTorqueFree, this, OwnThread)
		.doc("Change torque control mode of all servos.")
		.arg("torque_free", "If torque_free is true set TorqueFree mode (axises of servos are mannually movable). Use registers_init setting otherwise.");

	this->addOperation("publishJointStates", &HerkulexArray::publishJointStates, this, OwnThread)
		.doc("Read position and velocity of all servos and publish them on joints port. Return true on success.");

	this->addOperation("printServoStatus", &HerkulexArray::printServoStatus, this, OwnThread)
		.doc("Print status of servo in human readable format.")
		.arg("servo", "Servo name.");
	this->addOperation("printAllServoStatuses", &HerkulexArray::printAllServoStatuses, this, OwnThread)
		.doc("Print statuses of all servos.");
	this->addOperation("printErrorServoStatuses", &HerkulexArray::printErrorServoStatuses, this, OwnThread)
		.doc("Print statuses of erroneous servos.");
	this->addOperation("printRegisterRAM", &HerkulexArray::printRegisterRAM, this, OwnThread)
		.doc("Print register value in human readable format.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.");
	this->addOperation("printAllRegistersRAM", &HerkulexArray::printAllRegistersRAM, this, OwnThread)
		.doc("Print values of all registers in human readable format.")
		.arg("servo", "Servo name.");

	this->addOperation("discoverServos", &HerkulexArray::discoverServos, this, OwnThread)
		.doc("Find unregistered servos and add them to array.");

	// Prorocol access
	this->provides("protocol")->doc("Generation and parsing of HerkulexPackets");
	this->provides("protocol")->addOperation("reqIJOG", &HerkulexArray::reqIJOG, this, ClientThread)
		.doc("Generate IJOG packet, cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).")
		.arg("goal", "Position controlled servo new goal position (ServoGoal).");
	this->provides("protocol")->addOperation("reqPosVel", &HerkulexArray::reqPosVel, this, ClientThread)
		.doc("Generate READ packet for position and velocity query? cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).")
		.arg("servo", "Servo name.");
	this->provides("protocol")->addOperation("ackPosVel", &HerkulexArray::ackPosVel, this, ClientThread)
		.doc("Generate READ packet for position and velocity query, cause exeception on failure, return false on invalid packet.")
		.arg("ack", "Reference to received packet (HerkulexPacket).")
		.arg("servo", "Servo name.")
		.arg("pos", "Reference to position.")
		.arg("vel", "Reference to velocity.")
		.arg("status", "Servo status (byte0=status_error, byte1=status_detail).");
	this->provides("protocol")->addOperation("reqState", &HerkulexArray::reqState, this, ClientThread)
		.doc("Generate READ packet for extended state query (position, velocite, pwm, target, desired position and speed), cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).")
		.arg("servo", "Servo name.");
	this->provides("protocol")->addOperation("ackState", &HerkulexArray::ackState, this, ClientThread)
		.doc("Decode extended state query (position, velocite, pwm, target, desired position and speed), cause exeception on failure, return false on invalid packet.")
		.arg("ack", "Reference to received packet (HerkulexPacket).")
		.arg("servo", "Servo name.")
		.arg("state", "Reference to HerkulexServoState msg, if query has succed extended servo state will be pushed back.")
		.arg("status", "Servo status (byte0=status_error, byte1=status_detail).");
}

bool HerkulexArray::configureHook()
{

	servos.clear();
	servos_init.clear();
	break_loop_flag = false;

	//Read servos properties to HerkulexArray
    for(PropertyBag::const_iterator s = servos_prop.begin(); s != servos_prop.end(); s++) {
		// get servo description
        Property<PropertyBag> servo_prop(*s);
        if (!servo_prop.ready()) { 
            log(ERROR) << "Incorrect servos structure: all fist level elements must be PropertyBags." << endlog();
            return false;
        }

		std::string servo_name = servo_prop.getName();
        Property<unsigned int> servo_id_prop = servo_prop.rvalue().getProperty("servo_id");
        if (!servo_id_prop.ready()) { 
            log(ERROR) << "Incorrect servos structure: servo_id must be uint8." << endlog();
            return false;
        }
        /*Property<std::string> servo_model_prop = servo_prop.rvalue().getProperty("servo_model");
        if (!servo_model_prop.ready()) { 
            log(ERROR) << "Incorrect servos structure: servo_model must be string." << endlog();
            return false;
        }*/
        Property<bool> reverse_prop = servo_prop.rvalue().getProperty("reverse");
        if (!reverse_prop.ready()) { 
            log(ERROR) << "Incorrect servos structure: reverse must be bool." << endlog();
            return false;
        }
        Property<unsigned int> offset_prop = servo_prop.rvalue().getProperty("offset");
        if (!offset_prop.ready()) { 
            log(ERROR) << "Incorrect servos structure: offset must be int16." << endlog();
            return false;
        }

		/*if (servo_model_prop.rvalue() == "drs0101" || servo_model_prop.rvalue() == "drs0201") {
			servos.addServo(new HerkulexServoDRS101(name, servo_id_prop.rvalue(), reverse_prop.rvalue(), offset_prop.rvalue()));
		}
		else {
            log(ERROR) << "Incorrect servos structure: unknown servo model: " << servo_model_prop.rvalue() << ". Known models: drs101, drs202." << endlog();
			return false;
		}*/
		HerkulexServo * servo = new HerkulexServoDRS101(servo_name, servo_id_prop.rvalue(), reverse_prop.rvalue(), offset_prop.rvalue());
		log(DEBUG) << "Add servo name = " << servo->getName() << " servo_id = " << servo->getID() << endlog();
		if (!addServo(std::shared_ptr<HerkulexServo>(servo))) {
			log(ERROR) << "Incorrect servos structure: dublicate servo name or servo_id." << endlog();
			return false;
		}

		// Make cache for init registers values.
		RegisterValues * reg_init = new RegisterValues();
		servos_init[servo_name] = std::shared_ptr<RegisterValues>(reg_init);
		Property<PropertyBag> registers_init_prop = servo_prop.rvalue().getProperty("registers_init");
		if (registers_init_prop.ready()) {
			for(PropertyBag::const_iterator p = registers_init_prop.rvalue().begin(); p != registers_init_prop.rvalue().end(); p++) {
				Property<unsigned int> reg_val_prop(*p);
				if (!reg_val_prop.ready() || !servo->register_mapper.findByName(reg_val_prop.getName())) {
					log(ERROR) << "Incorrect servos structure: registers_init contains invalid property: " << servo_name << "." << reg_val_prop.getName() << endlog();
					return false;
				}
				reg_init->insert(RegisterValues::value_type(reg_val_prop.getName(), reg_val_prop.rvalue()));
				log(DEBUG) << "Cachce reg = " << reg_val_prop.getName() << " val = " << std::hex << std::setw(2) << std::setfill('0') << reg_val_prop.rvalue() << resetfmt << endlog();
			}
		}

	}

	// Check if "broadcast" object presents, remove it from HerkulexServoArray.
	HerkulexServoArray::const_iterator bcast = servos.find("broadcast");
	if (bcast == servos.end()) {
		log(ERROR) << "Incorrect servos structure: current version of HerkulexArray requeres brodcast servo record." << endlog();
		return false;
	}
	broadcast = bcast->second;
	broadcast_init = servos_init.at("broadcast");
	servos.erase(bcast);
	servos_init.erase("broadcast");

	//Prepare JointState an buffers.  Set sample to port.
	joints.name.resize(servos.size());
	joints.position.resize(servos.size());
	joints.velocity.resize(servos.size());

	req_pkt.data.resize(HerkulexPacket::DATA_SIZE);

	joints_port.setDataSample(joints);
	ack_buffer.data_sample(req_pkt);

	//Check if sendPacketCM is available.
	if (!sendPacketCM.ready()) {
		log(ERROR) << "sendPacketCM is not ready." << endlog();
		return false;
	}

	if (!resetAllServos()) return false;

	log(INFO) << "HerkulexArray is configured!" << endlog(); 
	return true;
}

const HerkulexServo& HerkulexArray::getServo(const string& name) 
{
	HerkulexServoArray::const_iterator s = servos.find(name);
	if (s != servos.end()) {
		return *(s->second);
	}
	else {
		throw std::out_of_range("getServo: servo " + name + " is not registered in array.");
	}
}

bool HerkulexArray::addServo(std::shared_ptr<HerkulexServo> servo) 
{
	// Check if servo name or HW ID is already occupaied
	if (servos.find(servo->getName()) != servos.end()) return false;
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) {
		if (servo->getID() == s->second->getID()) return false;
	}
	servos.insert( std::make_pair(servo->getName(), servo) );
	return true;
}

std::vector<std::string> HerkulexArray::listServos() 
{
	std::vector<std::string> list;
	list.reserve(servos.size());
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) list.push_back(s->first);
	return list;
}

std::vector<std::string> HerkulexArray::listServoRegistersRAM(const std::string& servo) 
{
	std::vector<std::string> list;
	try {
		const std::vector<Register>& registers = getServo(servo).register_mapper.registers;
		for(auto r = registers.begin(); r != registers.end(); r++) {
			if (r->ram_addr != -1) list.push_back(r->name);
		}
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
	}
	return list;
}

unsigned long HerkulexArray::getRegisterRAM(const std::string& servo, const std::string& reg) 
{
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int val;
		herkulex_servo::Status status;
		s.reqRead_ram(req_pkt, reg);
		bool success = sendRequest(req_pkt, s.ackCallbackRead_ram(reg, val, status));
		if (success) return val;
		else return READ_ERROR;
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return READ_ERROR;
	}
}

bool HerkulexArray::setRegisterRAM(const std::string& servo, const std::string& reg, unsigned int val)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		s.reqWrite_ram(req_pkt, reg, val);
		return sendRequest(req_pkt, s.ackCallbackWrite_ram(status));
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}

unsigned long HerkulexArray::getRegisterEEP(const std::string& servo, const std::string& reg) 
{
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int val;
		herkulex_servo::Status status;
		s.reqRead_eep(req_pkt, reg);
		bool success = sendRequest(req_pkt, s.ackCallbackRead_eep(reg, val, status));
		if (success) return val;
		else return READ_ERROR;
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return READ_ERROR;
	}
}

bool HerkulexArray::setRegisterEEP(const std::string& servo, const std::string& reg, unsigned int val)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		s.reqWrite_eep(req_pkt, reg, val);
		return sendRequest(req_pkt, s.ackCallbackWrite_eep(status));
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::setGoalRaw(const std::string& servo, unsigned int mode, unsigned int goal, unsigned int playtime)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		s.reqIJOGheader(req_pkt);
		s.insertIJOGdata(req_pkt, mode, goal, playtime);
		return sendRequest(req_pkt, s.ackCallbackIJOG(status));
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}
bool HerkulexArray::setGoal(const std::string& servo, unsigned int mode, double goal, double playtime)
{
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int goal_raw;
		if (mode & herkulex_servo::JOGMode::SPEED_CONTROL) goal_raw = s.convertVelRadToRaw(goal);
		else goal_raw = s.convertPosRadToRaw(goal);
		return setGoalRaw(servo, mode, goal_raw, s.convertTimeSecToRaw(playtime));
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}

unsigned long HerkulexArray::getStatus(const std::string& servo)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		s.reqStat(req_pkt);
		bool success =  sendRequest(req_pkt, s.ackCallbackStat(status));
		if (success) return status.error + (static_cast<unsigned long>(status.detail) << 8);
		else READ_ERROR;
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return READ_ERROR;
	}
}

bool HerkulexArray::clearStatus(const std::string& servo)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		s.reqWriteClearStatus(req_pkt);
		return sendRequest(req_pkt, s.ackCallbackWriteClearStatus(status));
	} 
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::setServoRegisters(const HerkulexServo * s, const RegisterValues * reg_init) 
{
	herkulex_servo::Status status;
	bool success = true;
	for(RegisterValues::const_iterator r = reg_init->begin(); r != reg_init->end(); r++) {
		s->reqWrite_ram(req_pkt, r->first, r->second);
		if (!sendRequest(req_pkt, s->ackCallbackWrite_ram(status))) {
			success = false;
		}
	}	
	return success;
}

bool HerkulexArray::resetServo(const std::string& servo)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		// RESET
		s.reqReset(req_pkt);
		bool success =  sendRequest(req_pkt, s.ackCallbackReset(status));
		//Wait for servo to resert.
		timeout_timer.arm(TIMEOUT_TIMER_ID, 1.0);
		timeout_timer.waitFor(TIMEOUT_TIMER_ID);
		// SET REPLY TO ALL
		s.reqWrite_ram(req_pkt, "ack_policy", 2);
		success = sendRequest(req_pkt, s.ackCallbackWrite_ram(status));
		if (!success) return false; 
		// INIT REGISTERS
		success = setServoRegisters(&s, broadcast_init.get()) && setServoRegisters(&s, servos_init.at(servo).get());
		return success;
	}
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::resetAllServos() 
{
	try {
		// reset all servos
		broadcast->reqReset(req_pkt);
		sendPacket(req_pkt);

		//Wait for servos to resert.
		timeout_timer.arm(TIMEOUT_TIMER_ID, 1.0);
		timeout_timer.waitFor(TIMEOUT_TIMER_ID);

		// Set ack policy: BROADCAST IS NOT WORKING
		//broadcast->reqWrite_ram(req_pkt, "ack_policy", 2);
		//sendPacket(req_pkt);
		// global registers init 
		/*const RegisterValues * reg_init = broadcast_init;
		for(RegisterValues::const_iterator r = reg_init->begin(); r != reg_init->end(); r++) {
			broadcast->reqWrite_ram(req_pkt, r->fist, r->second);
			sendPacketCM(req_pkt);
		}*/

		// init servo individually and check status
		herkulex_servo::Status status;
		bool success = true;
		for(HerkulexServoArray::const_iterator iter = servos.begin(); iter != servos.end(); iter++) {
			const HerkulexServo * s = iter->second.get();

			//set ack policy to always reply
			s->reqWrite_ram(req_pkt, "ack_policy", 2);
			if (!sendRequest(req_pkt, s->ackCallbackWrite_ram(status))) {
				log(ERROR) << "Write " << s->getName() << " \'ack_policy\' failed. Skipping servo." << endlog();
				success = false;
				continue;
			}

			if ( !setServoRegisters(s, broadcast_init.get()) || 
				 !setServoRegisters(s, servos_init.at(s->getName()).get()) ) 
			{
				log(ERROR) << "Write " << s->getName() << " servo registers failed." << endlog();
				success = false;
			}
			s->reqStat(req_pkt);
			if (sendRequest(req_pkt, s->ackCallbackStat(status))) {
				if (status.error & herkulex_servo::Status::ERROR_MASK) {
					log(WARN) << s->getName() << " ID = " << std::hex << s->getID() << std::dec << statusToString(status) << endlog();
				}
				//TODO invalid packet check
			}
			else {
				log(ERROR) << "Query " << s->getName() << " servo status failed." << endlog();
				success = false;
			}
		}
		return success;
	}
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::setAllServosTorqueFree(bool torque_free) 
{
	try {
		bool success = true;

		for(HerkulexServoArray::const_iterator iter = servos.begin(); iter != servos.end(); iter++) {
			const HerkulexServo * s = iter->second.get();
			unsigned int new_mode;

			if (torque_free) {
				new_mode = 0; 
			}
			else {
				herkulex_servo::RegisterValues::const_iterator reg_value_iter;
				const herkulex_servo::RegisterValues * reg_init = servos_init.at(s->getName()).get();
				reg_value_iter = reg_init->find("torque_control");
				if (reg_value_iter == reg_init->end()) {
					reg_value_iter = broadcast_init->find("torque_control");
					if (reg_value_iter == broadcast_init->end()) {
						// do nothin
						continue;
					}
				}
				new_mode = reg_value_iter->second;
			}

			herkulex_servo::Status status;
			s->reqWrite_ram(req_pkt, "torque_control", new_mode);
			if (!sendRequest(req_pkt, s->ackCallbackWrite_ram(status))) {
				log(ERROR) << "Write " << s->getName() << " \'torque_control\' failed. Skipping servo." << endlog();
				success = false;
			}
		}
		return success;
	}
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
		return false;
	}
}



bool HerkulexArray::publishJointStates()
{
	joints.name.clear();
	joints.position.clear();
	joints.velocity.clear();
	joints.effort.clear();
	bool success = true;
	unsigned int i = 0;
	double pos, vel;
	herkulex_servo::Status status;
	for(HerkulexServoArray::const_iterator iter = servos.begin(); iter != servos.end(); iter++) {
		const HerkulexServo& s = *(iter->second);
		s.reqPosVel(req_pkt);
		if (sendRequest(req_pkt, s.ackCallbackPosVel(pos, vel, status)))
		{
			joints.name.push_back(s.getName());
			joints.position.push_back(pos);
			joints.velocity.push_back(vel);
		}
		else {
			log(WARN) << "Read joint state for " << s.getName()<< " failed." << endlog();
			success = false;
		}
	}
	joints_port.write(joints);
	return success;
}

std::string HerkulexArray::statusToString(herkulex_servo::Status status) 
{
	std::stringstream status_str;

	if (status.detail & herkulex_servo::Status::MOTOR_ON) status_str << "ON ";
	else status_str << "OFF ";
	if (status.detail & herkulex_servo::Status::MOVING) status_str << "moving ";
	if (status.detail & herkulex_servo::Status::INPOSITION) status_str << "inpos ";
	if (status.error & herkulex_servo::Status::ERROR_MASK) {
		status_str << "ERR ( ";
		if (status.error & herkulex_servo::Status::ERROR_OVER_VOLTAGE) status_str << "voltage ";
		if (status.error & herkulex_servo::Status::ERROR_POT_LIMIT) status_str << "pot_limit ";
		if (status.error & herkulex_servo::Status::ERROR_TEMPERATURE) status_str << "temperature ";
		if (status.error & herkulex_servo::Status::ERROR_OVERLOAD) status_str << "overload ";
		if (status.error & herkulex_servo::Status::ERROR_DRIVER_FAULT) status_str << "driver_fault ";
		if (status.error & herkulex_servo::Status::ERROR_EEP_REGS) status_str << "eep_regs ";
		status_str << ") ";
	}
	else {
		status_str << "OK ";
	}
	if (status.error & herkulex_servo::Status::INVALID_PACKET) {
		status_str << "INVALID_PACKET ( ";
		if (status.detail & herkulex_servo::Status::INVALID_PACKET_CHECKSUM) status_str << "checksum ";
		if (status.detail & herkulex_servo::Status::INVALID_PACKET_UNKNOWN_CMD) status_str << "cmd ";
		if (status.detail & herkulex_servo::Status::INVALID_PACKET_REG_RANGE) status_str << "reg_range ";
		if (status.detail & herkulex_servo::Status::INVALID_PACKET_FRAME_ERROR) status_str << "frame_err ";
		status_str << ") ";
	};
	return status_str.str();
}

void HerkulexArray::printServoStatus(const std::string& servo)
{
	try {
		const HerkulexServo& s = getServo(servo);
		herkulex_servo::Status status;
		s.reqStat(req_pkt);
		bool success = sendRequest(req_pkt, s.ackCallbackStat(status));
		if (success) {
			std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << " \t" << statusToString(status) << std::endl;
		}
		else {
			log(ERROR) << "Unable query status of " << servo << " servo." << endlog();
			std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << " QUERY ERROR" << std::endl;
		}
	}
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
	}
}

void HerkulexArray::printAllServoStatuses()
{
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) {
		printServoStatus(s->first);
	}
}

void HerkulexArray::printErrorServoStatuses() 
{
	herkulex_servo::Status status;
	bool success;
	for(HerkulexServoArray::const_iterator iter = servos.begin(); iter != servos.end(); iter++) {
		const HerkulexServo& s = *(iter->second);
		s.reqStat(req_pkt);
		bool success = sendRequest(req_pkt, s.ackCallbackStat(status));
		if (success) {
			if (status.error & herkulex_servo::Status::ERROR_MASK) {
				std::cout << s.getName() << " ID = " << std::hex << s.getID() << std::dec << statusToString(status) << std::endl;
			}
		}
		else {
			log(ERROR) << "Unable query status of " << s.getName() << " servo." << endlog();
			std::cout << s.getName() << " ID = " << std::hex << s.getID() << std::dec << " QUERY ERROR" << std::endl;
		}
	}
}

void HerkulexArray::printRegisterRAM(const std::string& servo, const std::string& reg)
{
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned long result = getRegisterRAM(servo, reg);
		std::cout << s.getName() << " ID = " << std::hex << s.getID();
		if (result != READ_ERROR) {
			std::cout << reg << " = " << result << std::dec << std::endl;
		}
		else {
			std::cout << std::dec << " QUERY ERROR" << std::endl;
		}
	}
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
	}
}

void HerkulexArray::discoverServos() 
{
	for(unsigned int id = 0; id < 0xfe; id++) {
		// Check if ID presents in array.
		HerkulexServoArray::const_iterator s = servos.begin(); 
		while(s != servos.end()) {
			if (s->second->getID() == id) break;
			s++;
		}
		if (s != servos.end()) continue;
		// Create temporary object to access servo.
		std::shared_ptr<HerkulexServo> servo(new HerkulexServoDRS101("servo_id_" + std::to_string(id), id));
		// Request servo.
		herkulex_servo::Status status;
		servo->reqStat(req_pkt);
		bool success = sendRequest(req_pkt, servo->ackCallbackStat(status));
		if (!success) continue;

		// add servo to array
		if (!addServo(servo)) {
			std::cout << std::hex << "SERVO WITH ID = " <<  id << " IS FOUND." << std::endl;
			std::cout << "But servo with name '"  << servo->getID() << "' is already presents in array. Skipping." << std::endl << std::endl;
			continue;
		}
		servos_init[servo->getName()] = std::shared_ptr<RegisterValues>(new RegisterValues());
			
		// read servo info
		unsigned int model[2], version[2];
		success = resetServo(servo->getName())
			&& (model[0] = getRegisterEEP(servo->getName(), "model1")) != READ_ERROR
			&& (model[1] = getRegisterEEP(servo->getName(), "model2")) != READ_ERROR
			&& (version[0] = getRegisterEEP(servo->getName(), "version1")) != READ_ERROR
			&& (version[1] = getRegisterEEP(servo->getName(), "version2")) != READ_ERROR;
		// report results
		if (success) {
			std::cout << std::hex << "SERVO WITH ID = " << id << "(model: " << model[0] << model[1] << ", firmware version: " << version[0] << version[1] << ") IS FOUND." << std::endl;
			std::cout << "Servo is added to array under name '" << servo->getName() << "'."  << std::dec << std::endl ;
			std::cout << servo->getName() << " ID = " << std::hex << servo->getID() << std::dec << " \t" << statusToString(status) << std::endl;
			std::cout  << std::endl;
		}
		else {
			std::cout << std::hex << "SERVO WITH ID = " << id << " IS FOUND." << std::endl;
			std::cout << "Interaction with servo FAILED. Skipping."  << std::endl << std::endl;
			// remove servo from array
			servos.erase(servo->getName());
		}
	}
}
				
void HerkulexArray::printAllRegistersRAM(const std::string& servo) 
{
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int val;
		herkulex_servo::Status status;

		std::cout << s.getName() << " ID = " << std::hex << s.getID() << std::endl;
		for(auto r = s.register_mapper.registers.begin(); r != s.register_mapper.registers.end(); r++) {
			if (r->ram_addr != -1) {
				s.reqRead_ram(req_pkt, r->name);
				bool success = sendRequest(req_pkt, s.ackCallbackRead_ram(r->name, val, status));
				if (success) {
					std::cout << r->name << " = " << val << std::endl;
				}
				else {
					std::cout << r->name << " QUERY ERROR" << std::endl;
				}
			}
		}
		std::cout << std::dec << std::endl;
	}
	catch (const std::out_of_range& e) {
		log(ERROR) << e.what() << endlog();
	}
}

bool HerkulexArray::reqIJOG(HerkulexPacket& req, const sweetie_bot_hardware_herkulex_msgs::ServoGoal& goal)
{
	if (!this->isConfigured()) return false;
	if (goal.name.size() != goal.target_pos.size() && goal.name.size() != goal.playtime.size()) throw std::invalid_argument("HerkulexArray::reqIJOG: ServoGoal message has incorrect structure.");
	broadcast->reqIJOGheader(req);
	for(int i = 0; i < goal.name.size(); i++) {
		HerkulexServoArray::const_iterator s = servos.find(goal.name[i]);
		if (s != servos.end()) {
			s->second->insertIJOGdata(req, herkulex_servo::JOGMode::POSITION_CONTROL, s->second->convertPosRadToRaw(goal.target_pos[i]), s->second->convertTimeSecToRaw(goal.playtime[i]));
		}
	}
}

bool HerkulexArray::reqPosVel(HerkulexPacket& req, const std::string& servo)
{
	if (!this->isConfigured()) return false;
	getServo(servo).reqPosVel(req);
	return true;
}

bool HerkulexArray::ackPosVel(const HerkulexPacket& ack, const std::string& servo, double& pos, double& vel, unsigned int& _status) 
{
	if (!this->isConfigured()) return false;
	herkulex_servo::Status status;
	bool success = getServo(servo).ackPosVel(ack, pos, vel, status);
	_status = status;
	return success;
}

bool HerkulexArray::reqState(HerkulexPacket& req, const std::string& servo)
{
	if (!this->isConfigured()) return false;
	getServo(servo).reqState(req);
	return true;
}

bool HerkulexArray::ackState(const HerkulexPacket& ack, const std::string& servo, sweetie_bot_hardware_herkulex_msgs::HerkulexServoState& state_array, unsigned int& _status) 
{
	if (!this->isConfigured()) return false;
	herkulex_servo::Status status;
	herkulex_servo::State state;
	bool success = getServo(servo).ackState(ack, state, status);
	_status = status;
	if (success) {
		state_array.name.push_back(servo);
		state_array.pos.push_back(state.pos);
		state_array.vel.push_back(state.vel);
		state_array.pwm.push_back(state.pwm);
		state_array.pos_goal.push_back(state.pos_goal);
		state_array.pos_desired.push_back(state.pos_desired);
		state_array.vel_desired.push_back(state.vel_desired);
		state_array.status.push_back(_status);
	}
	return success;
}

void HerkulexArray::receivePacketCM(const HerkulexPacket& pkt) 
{
	ack_buffer.Push(pkt);
	ack_mutex.lock();
	ack_cond.broadcast();
	ack_mutex.unlock();
}

bool HerkulexArray::breakLoop() 
{
	ack_mutex.lock();
	break_loop_flag = true;
	ack_cond.broadcast();
	ack_mutex.unlock();
	return true;
}

void HerkulexArray::sendPacket(const HerkulexPacket& req) 
{
	if (log(DEBUG)) {
		log() << std::hex << std::setw(2) << std::setfill('0');
		log() << "REQ packet: servo_id: "  << (int) req.servo_id << " cmd: " << (int) req.command << " data(" << req.data.size() << "): ";
		for(auto c = req.data.begin(); c != req.data.end(); c++) log() << (int) *c << " ";
		log() << resetfmt << endlog();
	}
	sendPacketCM(req);
}


bool HerkulexArray::sendRequest(const HerkulexPacket& req, HerkulexServo::AckCallback ack_callback)
{
	HerkulexPacket * pkt_ack;
	unsigned int tryouts = tryouts_prop;
	bool success = false;

	do {
		timeout_timer.arm(TIMEOUT_TIMER_ID, timeout_prop);	
		sendPacket(req);
		tryouts--;
		do {
			ack_mutex.lock();
			while(ack_buffer.empty() && timeout_timer.isArmed(TIMEOUT_TIMER_ID) && !break_loop_flag ) ack_cond.wait(ack_mutex);
			ack_mutex.unlock();
			if (!ack_buffer.empty()) {
				pkt_ack = ack_buffer.PopWithoutRelease();
				//TODO invalid packet flag check
				success = ack_callback(*pkt_ack);

				if (log(DEBUG)) {
					log() << std::hex << std::setw(2) << std::setfill('0');
					log() << "ACK packet: servo_id: " << (int) pkt_ack->servo_id << " cmd: " << (int) pkt_ack->command << " data(" << std::dec << pkt_ack->data.size() << std::hex << "): ";
					for(auto c = pkt_ack->data.begin(); c != pkt_ack->data.end(); c++) log() << (int) *c << " ";
					log() << resetfmt << std::endl << "(success = " << success << ", tryout = " << tryouts_prop - tryouts << ")" << endlog();
				}

				ack_buffer.Release(pkt_ack);
			}
		} 
		while (!success && timeout_timer.isArmed(TIMEOUT_TIMER_ID) && !break_loop_flag);
		timeout_timer.killTimer(TIMEOUT_TIMER_ID);
		
		if (log(DEBUG)) {
			if (!success) {
				if (!break_loop_flag) log(DEBUG) << "ACK timeout" << endlog();
				else log(DEBUG) << "ACK break loop" << endlog();
			}
		}
	}
	while (!success && tryouts && !break_loop_flag);

	return success;
}	

bool HerkulexArray::startHook()
{
}

void HerkulexArray::updateHook()
{
	// updateHook is exected after all opertions are finished,
	break_loop_flag = false; 
}

void HerkulexArray::stopHook() 
{
}

void HerkulexArray::cleanupHook() 
{
	// clear servo properties cache
	servos.clear();
	servos_init.clear();
	// clear packet buffers
	ack_buffer.clear();
	ack_mutex.unlock();
	break_loop_flag = false;
	log(INFO) << "HerkulexArray is cleaned up!" << endlog();
}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HerkulexArray)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(sweetie_bot::HerkulexArray)
