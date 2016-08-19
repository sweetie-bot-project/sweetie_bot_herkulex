#include <iostream>

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include "herkulex_array.hpp"

#include "herkulex_servo.hpp"
#include "herkulex_servo_drs101.hpp"

using namespace herkulex_servo;
using namespace RTT;

//TODO Where should be this code?
std::ostream& resetfmt(std::ostream& s) {
	s.copyfmt(std::ios(NULL)); 
	return s;
}

const unsigned long HerkulexArray::READ_ERROR = 0x10000;
const unsigned int HerkulexArray::JOG_STOP = HerkulexServo::JOGMode::STOP;
const unsigned int HerkulexArray::JOG_POSITION = HerkulexServo::JOGMode::POSITION_CONTROL;
const unsigned int HerkulexArray::JOG_SPEED = HerkulexServo::JOGMode::SPEED_CONTROL;

HerkulexArray::HerkulexArray(std::string const& name) : 
	TaskContext(name, PreOperational),
	sendPacketCM("sendPacketCM"),
	ack_buffer(10, HerkulexPacket(), true), // circular buffer for 10 packets
	timeout_timer(this),
	break_loop_flag(false)
{
	Logger::In("HerkulexPacket");

	// BASIC INITIALIZATION
	// Start timer thread.
	if (!timeout_timer.getActivity() || !timeout_timer.getActivity()->thread()) {
		log(Error) << "Unable to start timer thread.";
		this->fatal();
		return;
	}
	timeout_timer.getActivity()->thread()->start();
	// init brodcast object
	broadcast = std::shared_ptr<HerkulexServo>(new HerkulexServoDRS101("broadcast", 0xFE, false, 0));

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
			 "\t\t\t    PropertyBag servo_name1 { string hw_id, string servo_model, uint offset, bool reverse, PropertyBag register_init { uint register1, uint register2, ... } },\n"
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
	this->addPort("joints", joints_port).doc("Output JointState by request.");

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

	this->addOperation("publishJointStates", &HerkulexArray::publishJointStates, this, OwnThread)
		.doc("Read position and velocity of all servos and publish them on joints port. Return true on success.");

	this->addOperation("printServoStatus", &HerkulexArray::printServoStatus, this, OwnThread)
		.doc("Print status of servo in human readable format.")
		.arg("servo", "Servo name.");
	this->addOperation("printAllServoStatuses", &HerkulexArray::printAllServoStatuses, this, OwnThread)
		.doc("Print statuses of all servos.");
	this->addOperation("printErrorServoStatuses", &HerkulexArray::printAllServoStatuses, this, OwnThread)
		.doc("Print statuses of erroneous servos.");
	this->addOperation("printRegisterRAM", &HerkulexArray::printRegisterRAM, this, OwnThread)
		.doc("Print register value in human readable format.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.");
	this->addOperation("printAllRegistersRAM", &HerkulexArray::printAllRegistersRAM, this, OwnThread)
		.doc("Print values of all registers in human readable format.")
		.arg("servo", "Servo name.");

	// Prorocol access
	/*this->provides("protocol")->addOperation("reqIJOG", &HerkulexArray::reqIJOG, this, ClientThread)
		.doc("Generate IJOG packet, cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).");
		.arg("goal", "Position controlled servo new goal position (ServoGoal).");*/
	this->provides("protocol")->doc("Generation and parsing of HerkulexPackets");
	this->provides("protocol")->addOperation("reqPosVel", &HerkulexArray::reqPosVel, this, ClientThread)
		.doc("Generate READ packet for position and velocity query? cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).")
		.arg("servo", "Servo name.");
	this->provides("protocol")->addOperation("acqPosVel", &HerkulexArray::ackPosVel, this, ClientThread)
		.doc("Generate READ packet for position and velocity query, cause exeception on failure, return false on invalid packet.")
		.arg("ack", "Reference to received packet (HerkulexPacket).")
		.arg("servo", "Servo name.")
		.arg("pos", "Reference to position.")
		.arg("vel", "Reference to velocity.");
}

bool HerkulexArray::configureHook()
{
	Logger::In("HerkulexArray");

	servos.clear();
	servos_init.clear();
	break_loop_flag = false;

	//Read servos properties to HerkulexArray
    for(PropertyBag::const_iterator s = servos_prop.begin(); s != servos_prop.end(); s++) {
		// get servo description
        Property<PropertyBag> servo_prop(*s);
        if (!servo_prop.ready()) { 
            log(Error) << "Incorrect servos structure: all fist level elements must be PropertyBags." << endlog();
            return false;
        }

		std::string servo_name = servo_prop.getName();
        Property<unsigned int> hw_id_prop = servo_prop.rvalue().getProperty("hw_id");
        if (!hw_id_prop.ready()) { 
            log(Error) << "Incorrect servos structure: hw_id must be uint8." << endlog();
            return false;
        }
        /*Property<std::string> servo_model_prop = servo_prop.rvalue().getProperty("servo_model");
        if (!servo_model_prop.ready()) { 
            log(Error) << "Incorrect servos structure: servo_model must be string." << endlog();
            return false;
        }*/
        Property<bool> reverse_prop = servo_prop.rvalue().getProperty("reverse");
        if (!reverse_prop.ready()) { 
            log(Error) << "Incorrect servos structure: reverse must be bool." << endlog();
            return false;
        }
        Property<unsigned int> offset_prop = servo_prop.rvalue().getProperty("offset");
        if (!offset_prop.ready()) { 
            log(Error) << "Incorrect servos structure: offset must be int16." << endlog();
            return false;
        }

		/*if (servo_model_prop.rvalue() == "drs0101" || servo_model_prop.rvalue() == "drs0201") {
			servos.addServo(new HerkulexServoDRS101(name, hw_id_prop.rvalue(), reverse_prop.rvalue(), offset_prop.rvalue()));
		}
		else {
            log(Error) << "Incorrect servos structure: unknown servo model: " << servo_model_prop.rvalue() << ". Known models: drs101, drs202." << endlog();
			return false;
		}*/
		HerkulexServo * servo = new HerkulexServoDRS101(servo_name, hw_id_prop.rvalue(), reverse_prop.rvalue(), offset_prop.rvalue());
		log(Debug) << "Add servo name = " << servo->getName() << " hw_id = " << servo->getID() << endlog();
		if (!addServo(std::shared_ptr<HerkulexServo>(servo))) {
			log(Error) << "Incorrect servos structure: dublicate servo name or hw_id." << endlog();
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
					log(Error) << "Incorrect servos structure: registers_init contains invalid property: " << servo_name << "." << reg_val_prop.getName() << endlog();
					return false;
				}
				reg_init->insert(RegisterValues::value_type(reg_val_prop.getName(), reg_val_prop.rvalue()));
				log(Debug) << "Cachce reg = " << reg_val_prop.getName() << " val = " << std::hex << std::setw(2) << std::setfill('0') << reg_val_prop.rvalue() << resetfmt << endlog();
			}
		}

	}

	// Check if "broadcast" object presents, remove it from HerkulexServoArray.
	HerkulexServoArray::const_iterator bcast = servos.find("broadcast");
	if (bcast == servos.end()) {
		log(Error) << "Incorrect servos structure: current version of HerkulexArray requeres brodcast servo record." << endlog();
		return false;
	}
	broadcast = bcast->second;
	servos.erase(bcast);

	//Prepare JointState an buffers.  Set sample to port.
	joints.name.resize(servos.size());
	joints.position.resize(servos.size());
	joints.velocity.resize(servos.size());

	req_pkt.data.resize(HerkulexPacket::DATA_SIZE);

	joints_port.setDataSample(joints);
	ack_buffer.data_sample(req_pkt);

	//Check if sendPacketCM is available.
	if (!sendPacketCM.ready()) {
		log(Error) << "sendPacketCM is not ready." << endlog();
		return false;
	}

	return resetAllServos();
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
	Logger::In("HerkulexArray");
	std::vector<std::string> list;
	try {
		const std::vector<Register>& registers = getServo(servo).register_mapper.registers;
		for(auto r = registers.begin(); r != registers.end(); r++) {
			if (r->ram_addr != -1) list.push_back(r->name);
		}
	} 
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
	}
	return list;
}

unsigned long HerkulexArray::getRegisterRAM(const std::string& servo, const std::string& reg) 
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int val;
		HerkulexServo::Status status;
		s.reqRead_ram(req_pkt, reg);
		bool success = sendRequest(req_pkt, s.ackCallbackRead_ram(reg, val, status));
		if (success) return val;
		else return READ_ERROR;
	} 
	catch (const std::out_of_range& e) {
		Logger::In("HerkulexArray");
		log(Error) << e.what() << endlog();
		return READ_ERROR;
	}
}

bool HerkulexArray::setRegisterRAM(const std::string& servo, const std::string& reg, unsigned int val)
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		HerkulexServo::Status status;
		s.reqWrite_ram(req_pkt, reg, val);
		return sendRequest(req_pkt, s.ackCallbackWrite_ram(status));
	} 
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::setGoalRaw(const std::string& servo, unsigned int mode, unsigned int goal, unsigned int playtime)
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		HerkulexServo::Status status;
		s.reqIJOGheader(req_pkt);
		s.insertIJOGdata(req_pkt, mode, goal, playtime);
		return sendRequest(req_pkt, s.ackCallbackIJOG(status));
	} 
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return false;
	}
}
bool HerkulexArray::setGoal(const std::string& servo, unsigned int mode, double goal, double playtime)
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int goal_raw;
		if (mode & HerkulexServo::JOGMode::SPEED_CONTROL) goal_raw = s.convertVelRadToRaw(goal);
		else goal_raw = s.convertPosRadToRaw(goal);
		return setGoalRaw(servo, mode, goal_raw, s.convertTimeSecToRaw(playtime));
	} 
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return false;
	}
}

unsigned long HerkulexArray::getStatus(const std::string& servo)
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		HerkulexServo::Status status;
		s.reqStat(req_pkt);
		bool success =  sendRequest(req_pkt, s.ackCallbackStat(status));
		if (success) return status.error + (static_cast<unsigned long>(status.detail) << 8);
		else READ_ERROR;
	} 
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return READ_ERROR;
	}
}

bool HerkulexArray::clearStatus(const std::string& servo)
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		HerkulexServo::Status status;
		s.reqWriteClearStatus(req_pkt);
		return sendRequest(req_pkt, s.ackCallbackWriteClearStatus(status));
	} 
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::setServoRegisters(const HerkulexServo * s, const RegisterValues * reg_init) 
{
	HerkulexServo::Status status;
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
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		HerkulexServo::Status status;
		// RESET
		s.reqReset(req_pkt);
		bool success =  sendRequest(req_pkt, s.ackCallbackReset(status));
		//if (!success) return false; 
		// SET REPLY TO ALL
		s.reqWrite_ram(req_pkt, "ack_policy", 2);
		success = sendRequest(req_pkt, s.ackCallbackWrite_ram(status));
		if (!success) return false; 
		// INIT REGISTERS
		success = setServoRegisters(&s, servos_init.at("broadcast").get()) && setServoRegisters(&s, servos_init.at(servo).get());
		return success;
	}
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::resetAllServos() 
{
	Logger::In("HerkulexArray");
	try {
		// reset all servos
		broadcast->reqReset(req_pkt);
		sendPacket(req_pkt);
		// set ack policy
		broadcast->reqWrite_ram(req_pkt, "ack_policy", 2);
		sendPacket(req_pkt);
		// global registers init (REMOVED: better set registers value individually)
		/*const RegisterValues * reg_init = servos_init.at("broadcast");
		for(RegisterValues::const_iterator r = reg_init->begin(); r != reg_init->end(); r++) {
			broadcast->reqWrite_ram(req_pkt, r->fist, r->second);
			sendPacketCM(req_pkt);
		}*/
		// init servo individually and check status
		HerkulexServo::Status status;
		bool success = true;
		for(HerkulexServoArray::const_iterator iter = servos.begin(); iter != servos.end(); iter++) {
			const HerkulexServo * s = iter->second.get();
			if ( !setServoRegisters(s, servos_init.at("broadcast").get()) || 
				 !setServoRegisters(s, servos_init.at(s->getName()).get()) ) 
			{
				log(Error) << "Write " << s->getName() << " servo registers failed." << endlog();
				success = false;
			}
			s->reqStat(req_pkt);
			if (sendRequest(req_pkt, s->ackCallbackStat(status))) {
				if (status.error & HerkulexServo::Status::ERROR_MASK) {
					std::cout << s->getName() << " ID = " << std::hex << s->getID() << std::dec << statusToString(status) << std::endl;
					success = false;
				}
			}
			else {
				log(Error) << "Query " << s->getName() << " servo status failed." << endlog();
				success = false;
			}
		}
		return success;
	}
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
		return false;
	}
}

bool HerkulexArray::publishJointStates()
{
	Logger::In("HerkulexArray");
	joints.name.clear();
	joints.position.clear();
	joints.velocity.clear();
	joints.effort.clear();
	bool success = true;
	unsigned int i = 0;
	double pos, vel;
	HerkulexServo::Status status;
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
			log(Warning) << "Read joint state for " << s.getName()<< " failed." << endlog();
			success = false;
		}
	}
	joints_port.write(joints);
	return success;
}

std::string HerkulexArray::statusToString(HerkulexServo::Status status) 
{
	std::stringstream status_str;

	if (status.detail & HerkulexServo::Status::MOTOR_ON) status_str << "ON ";
	else status_str << "OFF ";
	if (status.detail & HerkulexServo::Status::MOVING) status_str << "moving ";
	if (status.detail & HerkulexServo::Status::INPOSITION) status_str << "inpos ";
	if (status.error & HerkulexServo::Status::ERROR_MASK) {
		status_str << "ERR ( ";
		if (status.error & HerkulexServo::Status::ERROR_OVER_VOLTAGE) status_str << "voltage ";
		if (status.error & HerkulexServo::Status::ERROR_POT_LIMIT) status_str << "pot_limit ";
		if (status.error & HerkulexServo::Status::ERROR_TEMPERATURE) status_str << "temperature ";
		if (status.error & HerkulexServo::Status::ERROR_OVERLOAD) status_str << "overload ";
		if (status.error & HerkulexServo::Status::ERROR_DRIVER_FAULT) status_str << "driver_fault ";
		if (status.error & HerkulexServo::Status::ERROR_EEP_REGS) status_str << "eep_regs ";
		status_str << ") ";
	}
	else {
		status_str << "OK ";
	}
	if (status.error & HerkulexServo::Status::INVALID_PACKET) {
		status_str << "INVALID_PACKET ( ";
		if (status.detail & HerkulexServo::Status::INVALID_PACKET_CHECKSUM) status_str << "checksum ";
		if (status.detail & HerkulexServo::Status::INVALID_PACKET_UNKNOWN_CMD) status_str << "cmd ";
		if (status.detail & HerkulexServo::Status::INVALID_PACKET_REG_RANGE) status_str << "reg_range ";
		if (status.detail & HerkulexServo::Status::INVALID_PACKET_FRAME_ERROR) status_str << "frame_err ";
		status_str << ") ";
	};
	return status_str.str();
}

void HerkulexArray::printServoStatus(const std::string& servo)
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		HerkulexServo::Status status;
		s.reqStat(req_pkt);
		bool success = sendRequest(req_pkt, s.ackCallbackStat(status));
		if (success) {
			std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << statusToString(status) << std::endl;
		}
		else {
			log(Error) << "Unable query status of " << servo << " servo." << endlog();
			std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << " QUERY ERROR" << std::endl;
		}
	}
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
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
	Logger::In("HerkulexArray");
	HerkulexServo::Status status;
	bool success;
	for(HerkulexServoArray::const_iterator iter = servos.begin(); iter != servos.end(); iter++) {
		const HerkulexServo& s = *(iter->second);
		s.reqStat(req_pkt);
		bool success = sendRequest(req_pkt, s.ackCallbackStat(status));
		if (success) {
			if (status.error & HerkulexServo::Status::ERROR_MASK) {
				std::cout << s.getName() << " ID = " << std::hex << s.getID() << std::dec << statusToString(status) << std::endl;
			}
		}
		else {
			log(Error) << "Unable query status of " << s.getName() << " servo." << endlog();
			std::cout << s.getName() << " ID = " << std::hex << s.getID() << std::dec << " QUERY ERROR" << std::endl;
		}
	}
}

void HerkulexArray::printRegisterRAM(const std::string& servo, const std::string& reg)
{
	Logger::In("HerkulexArray");
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
		log(Error) << e.what() << endlog();
	}
}


void HerkulexArray::printAllRegistersRAM(const std::string& servo) 
{
	Logger::In("HerkulexArray");
	try {
		const HerkulexServo& s = getServo(servo);
		unsigned int val;
		HerkulexServo::Status status;

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
				std::cout << std::dec << std::endl;
			}
		}
	}
	catch (const std::out_of_range& e) {
		log(Error) << e.what() << endlog();
	}
}

/*
bool HerkulexArray::reqIJOG(HerkulexPacket& req, const sweetie_bot_core_msgs::ServoGoal& goal)
{
	if (!this->isConfigured()) return false;
	if (goal.name.size() != goal.target_position) throw std::bad_arg("HerkulexArray::reqIJOG: ServoGoal message has incorrect structure.");
	broadcast.reqIJOGheader(req);
	for(int i = 0; i < goal.name.size(); i++) {
		const HerkulexServo * s = servo.findByName(goal.name[i]);
		if (s != nullptr) {
			s->insertSJOGdata(req, HerkulexServo::JOGMode::POSITION_CONTROL, s->convertPosRadToRaw(goal.target_position[i]));
		}
	}
}
*/

bool HerkulexArray::reqPosVel(HerkulexPacket& req, const std::string servo)
{
	if (!this->isConfigured()) return false;
	getServo(servo).reqPosVel(req);
	return true;
}

bool HerkulexArray::ackPosVel(HerkulexPacket& ack, const std::string servo, double& pos, double& vel) 
{
	if (!this->isConfigured()) return false;
	HerkulexServo::Status status;
	return getServo(servo).ackPosVel(ack, pos, vel, status);
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
	if (log().getLogLevel() >= Logger::Debug) {
		log() << Logger::Debug << std::hex << std::setw(2) << std::setfill('0');
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
				success = ack_callback(*pkt_ack);

				if (log().getLogLevel() >= Logger::Debug) {
					log() << Logger::Debug << std::hex << std::setw(2) << std::setfill('0');
					log() << "ACK packet: servo_id: " << (int) pkt_ack->servo_id << " cmd: " << (int) pkt_ack->command << " data(" << pkt_ack->data.size() << "): ";
					for(auto c = pkt_ack->data.begin(); c != pkt_ack->data.end(); c++) log() << (int) *c << " ";
					log() << resetfmt << Logger::nl << "(success = " << success << ", tryout = " << tryouts_prop - tryouts << ")" << endlog();
				}

				ack_buffer.Release(pkt_ack);
			}
		} 
		while (!success && timeout_timer.isArmed(TIMEOUT_TIMER_ID) && !break_loop_flag);
		timeout_timer.killTimer(TIMEOUT_TIMER_ID);
		
		if (log().getLogLevel() >= Logger::Debug) {
			if (!success) {
				if (!break_loop_flag) log(Debug) << "ACK timeout" << endlog();
				else log(Debug) << "ACK break loop" << endlog();
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

void HerkulexArray::cleanupHook() {
	// clear servo properties cache
	servos.clear();
	servos_init.clear();
	// clear packet buffers
	ack_buffer.clear();
	ack_mutex.unlock();
	break_loop_flag = false;
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
ORO_CREATE_COMPONENT(HerkulexArray)
