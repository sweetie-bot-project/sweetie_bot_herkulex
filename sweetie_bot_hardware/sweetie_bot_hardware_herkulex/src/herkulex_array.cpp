#include "herkulex_driver.hpp"

#include <rtt/Component.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <rtt/Logger.hpp>


using namespace RTT;
using namespace herkulex_servo;

/*
#define CMD_REQ_ACK_RETRY(req_expression, ack_expression, tryouts)    \
    ({ bool __success; int __tryout = (tryouts);  \
	   (req_expression);						  \
       do {										  \
	       cmdSync(pkt_req; pkt_ack);			  \
	       __success = (expression);              \
	       __tryout--; }						  \
       while (__tryout && !__success);			  \
       __success; })
*/

static const unsigned long HerkulexArray::READ_ERROR = 0x10000;

HerkulexArray::HerkulexArray(std::string const& name) : 
	TaskContext(name),
	sendPacketCM("sendPacketCM"),
{
	// CONSTANTS
	this->addConstant("READ_ERROR", READ_ERROR).
		doc("Error code for getRegister() operation.");

	// PROPERTIES
		RTT::PropertyBag servos_prop;
		unsigned int tryouts_prop;
		double timeout;
	this->addProperty("servos", servos_prop) 
		.doc("Servo descriptions (PropertuBag). Format:
{ 
	PropertyBag servo_name1 { string hw_id, string servo_model, uint offset, bool reverse, PropertyBag register_init { uint register1, uint register2, ... } },
    PropertyBag servo_name2 { ... }	
	...
}");
	this->addProperty("tryouts", tryouts_prop)
		.doc("Number of attemts to perform one operation.")
		.set(3);
	this->addProperty("timeout", timeout_prop)
		.doc("Operation timeout (s).")
		.set(0.01);

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
		.doc("Return the list of registered servos");
	this->addOperation("listRegisters", &HerkulexArray::listRegisters, this, ClientThread)
		.doc("Return the list of servo registers.");

	this->addOperation("setRegisterRAM", &HerkulexArray::setRegisterRAM, this, OwnThread)
		.doc("Set servo register value. Return true on success.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.")
		.arg("val", "New register value (raw uint16).");
	this->addOperation("getRegisterRAM", &HerkulexArray::getRegisterRAM, this, OwnThread)
		.doc("Get servo register value. Return register raw value, on error return READ_ERROR.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.");

	this->addOperation("getStatus", &HerkulexArray::getStatus, this, OwnThread)
		.doc("Get servo status. Return content of status registers (byte0=status_error, byte1=status_detail) or READ_ERROR.")
		.arg("servo", "Servo name.");
	this->addOperation("clearStatus", &HerkulexArray::clearStatus, this, OwnThread)
		.doc("Clear status register of servo. Return true on success.")
		.arg("servo", "Servo name.");
	this->addOperation("resetServo", &HerkulexArray::resetServo, this, OwnThread)
		.doc("Reset servo and init it again. Must clear all *hard* error status.")
		.arg("servo", "Servo name.");

	this->addOperation("getJointStates", &HerkulexArray::getJointStates, this, OwnThread)
		.doc("Read position and velocity of all servos and publish them on joints port. Return true on success.").

	this->addOperation("printServoStatus", &HerkulexArray::printServoStatus, this, OwnThread)
		.doc("Print status of servo in human readable format.")
		.arg("servo", "Servo name.");
	this->addOperation("printAllServoStatuses", &HerkulexArray::printAllServoStatuses, this, OwnThread)
		.doc("Print statuses of all servos.");
	this->addOperation("printErrorServoStatuses", &HerkulexArray::printAllServoStatuses, this, OwnThread)
		.doc("Print statuses of erroneous servos.");
	this->addOperation("printRegisterRAM", &HerkulexArray::printRegister, this, OwnThread)
		.doc("Print register value in human readable format.")
		.arg("servo", "Servo name.")
		.arg("reg", "Register.");
	this->addOperation("printAllRegistersRAM", &HerkulexArray::printAllRegisters, this, OwnThread)
		.doc("Print values of all registers in human readable format.")
		.arg("servo", "Servo name.");

	// Prorocol access
	this->provides("protocol")->addOperation("reqIJOG", &HerkulexArray::reqIJOG, this, ClientThread)
		.doc("Generate IJOG packet, cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).");
		.arg("goal", "Position controlled servo new goal position (ServoGoal).");
	this->provides("protocol")->addOperation("reqPosVel", &HerkulexArray::reqPosVel, this, ClientThread)
		.doc("Generate READ packet for position and velocity query? cause exeception on failure.")
		.arg("req", "Reference to generated packet (HerkulexPacket).");
		.arg("servo", "Servo name.");
	this->provides("protocol")->addOperation("acqPosVel", &HerkulexArray::ackPosVel, this, ClientThread)
		.doc("Generate READ packet for position and velocity query, cause exeception on failure, return false on invalid packet.")
		.arg("ack", "Reference to received packet (HerkulexPacket).");
		.arg("servo", "Servo name.");
		.arg("pos", "Reference to position.");
		.arg("vel", "Reference to velocity.");
}

bool HerkulexArray::configureHook()
{
	servos.clear();
	servos_init.clear();

	//Read servos properties to HerkulexArray
    for(PropertyBag::const_iterator s = servos_prop.rvalue().begin(); s != < servos_prop.rvalue().end; s++) {
		// get servo description
        Property<PropertyBag> servo_prop(*s);
        if (!servo_prop.ready()) { 
            log(Error) << "Incorrect servos structure: all fist level elements must be PropertyBags." << endlog();
            return false;
        }

        std::string servo_name = servo_prop.getName();
        Property<unsigned char> hw_id_prop = servo_prop.rvalue().getProperty("servo_model");
        if (!hw_id_prop.ready()) { 
            log(Error) << "Incorrect servos structure: hw_id must be unsigned char." << endlog();
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
            log(Error) << "Incorrect servos structure: offset must be unsigned int." << endlog();
            return false;
        }

		/*if (servo_model_prop.rvalue() == "drs0101" || servo_model_prop.rvalue() == "drs0201") {
			servos.addServo(new HerkulexServoDRS101(name, hw_id_prop.rvalue(), reverse_prop.rvalue(), offset_prop.rvalue()));
		}
		else {
            log(Error) << "Incorrect servos structure: unknown servo model: " << servo_model_prop.rvalue() << ". Known models: drs101, drs202." << endlog();
			return false;
		}*/
		if (!servos.addServo(new HerkulexServoDRS101(name, hw_id_prop.rvalue(), reverse_prop.rvalue(), offset_prop.rvalue()))) {
			log(Error) << "Incorrect servos structure: dublicate servo name or hw_id." << endlog();
		}

		// Make cache for init registers values.
		RegisterValues * reg_init;
		reg_init = servos_init[name] = new RegisterValues();
		Property<PropertyBag> registers_init_prop = servos_prop.rvalue().getProperty("registers_init");
		if (registers_init_prop.ready()) {
			for(PropertyBag::const_iterator p = registers_init_prop.rvalue().begin(); p != registers_init_prop.rvalue().end(); p++) {
				Property<unsigned int> reg_val_prop(*p);
				if (!reg_val_prop.ready() || !servo->register_mapper.findByName(reg_val_prop.getName())) {
					log(Error) << "Incorrect servos structure: registers_init contains invalid property: " << name << "." << reg_val_prop.getName() << endlog();
					return false;
				}
				reg_init->insert(RegisterValues::value_type(reg_val_prop.getName(), reg_val_prop.rvalue()));
			}
		}

	}

	// Check if "broadcast" object presents.
	HerkulexArray::const_iterator bcast_ptr = servos.find("broadcast");
	if (bcast == servos.end()) {
		log(Error) << "Incorrect servos structure: current version of HerkulexArray requeres brodcast servo record." << endlog();
		return false;
	}
	broadcast = *bcast_ptr;
	servos.erase(bcast_ptr);

	//Prepare JointState and set sample to port
	joints.name.reserve(servos

	//Perform servo initialization.
	//1. Reset servos (use broadcast).
	//2. Set reply mode (use broadcast).
	//3. Check servos statuses.
	//4. Set registers.
	return resetAllServos();
}

std::vector<std::string> HerkulexArray::listServos() 
{
	std::vector<std::string> list;
	list.reserve(servos.size());
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) list.push_back(s->fist);
}

std::vector<std::string> HerkulexArray::listRegisters(const std::string& servo) 
{
	std::vector<std::string> list;
	try {
		list = servos.getServo(servo).register_mapper.registers;
	} 
	catch (const std::out_of_range& e) {
		log(Error) << "Register or servo does not exists: " << e.what() << endlog();
	}
	return list;
}

unsigned long HerkulexArray::getRegisterRAM(const std::string& servo, const std::string& reg) 
{
	const HerkulexServo& s(servos.getServo(servo));
	unsigned long val;
	Status status;
	bool success = CMD_REQ_ACK_RETRY(s.reqRead_ram(pkt_req, reg), s.ackRead_ram(pkt_ack, reg, val, status), tryouts_prop.rvalue());
	if (success) return val;
	else return READ_ERROR;
}

bool HerkulexArray::setRegisterRAM(const std::string& servo, const std::string& reg, unsigned int val)
{
	const HerkulexServo& s(servos.getServo(servo));
	Status status;
	return CMD_REQ_ACK_RETRY(s.reqWrite_ram(pkt_req, reg, val), s.ackRead_ram(pkt_ack, reg, status), tryouts_prop.rvalue());
}

unsigned long HerkulexArray::getStatus(const std::string& servo)
{
	const HerkulexServo& s(servos.getServo(servo_name));
	HerkulexServo::Status status;
	bool success = CMD_REQ_ACK_RETRY(s.reqStat(pkt_req), s.ackStat(pkt_ack, status), tryouts_prop.rvalue());
	if (success) return status.error + (static_cast<unsigned long>(status.detail) << 8);
	else READ_ERROR;
}

bool HerkulexArray::clearStatus(const std::string& servo)
{
	const HerkulexServo& s(servos.getServo(servo_name));
	HerkulexServo::Status status;
	return CMD_REQ_ACK_RETRY(s.reqWriteClearStatus(pkt_req), s.ackWriteClearStatus(pkt_ack, status), tryouts_prop.rvalue())
}

bool HerkulexArray::resetServo(const std::string& servo)
{
	const HerkulexServo& s(servos.getServo(servo));
	HerkulexServo::Status status;
	// RESET
	bool success = CMD_REQ_ACK_RETRY(s.reqReset(pkt_req), s.ackReset(pkt_ack, status) || pkt_ack.command == HerkulexPacket::ERR_TIMEOUT, tryouts_prop.rvalue()) 
	if (!success) return false; 
	// SET REPLY TO ALL
	success = CMD_REQ_ACK_RETRY(s.reqWrite_ram(pkt_req, "ack_policy", 2), s.ackWrite_ram(pkt_ack, status), tryouts_prop.rvalue());
	if (!success) return false; 
	// INIT REGISTERS
	const RegisterValues * reg_init = servos_init.at(servo);
	for(RegisterValues::const_iterator r = reg_init.begin(); r != reg_init.end(); r++) {
		if (!CMD_REQ_ACK_RETRY(s.reqWrite_ram(pkt_req, r->fist, r->second), s.ackWrite_ram(pkt_ack, status)), tryouts_prop.rvalue()) {
			success = false;
		}
	}	
	return success;
}

void HerkulexArray::resetAllServos() 
{
	bool success = true;
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) {
		if (!resetServo(s->fist)) {
			log(Warning) << "Reset servo " << s->fist << " failed." << endlog();
			success = false;
		}
	}
	return success;
}

HerkulexArray::getJointStates()
{
	joints.name.clear();
	joints.position.clear();
	joints.velocity.clear();
	joints.effort.clear();
	bool success = true;
	unsigned int i = 0;
	double pos, vel;
	HerkulexServo::Status status;
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) {
		if (CMD_REQ_ACK_RETRY(s->reqPosVel(pkt_req), s->ackPosVel(pkt_ack, joints, pos, vel, status), tryouts_prop.rvalue()))
		{
			joints.name.push_back(s->fist);
			joints.position.push_back(pos);
			joints.velocity.push_back(vel);
		}
		else {
			log(Warning) << "Read joint state for " << s->fist << " failed." << endlog();
			success = false;
		}

	}
	joints_port.write(joints);
	return success;
}

std::string HerkulexArray::statusToString(Status status) 
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
	return status_str;
}


void HerkulexArray::printServoStatus(const std::string& servo)
{
	const HerkulexServo& s(servos.getServo(servo));
	HerkulexServo::Status status;
	bool success = CMD_REQ_ACK_RETRY(s.reqStat(pkt_req), s.ackStat(pkt_ack, status), tryouts_prop.rvalue());
	if (success) {
		std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << statusToString(status) << std::endl;
	}
	else {
		log(Error) << "Unable query status of " << servo << " servo." << endlog();
		std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << " QUERY ERROR" << std::endl;
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
	HerkulexServo::Status status;
	bool success;
	for(HerkulexServoArray::const_iterator s = servos.begin(); s != servos.end(); s++) {
		success = CMD_REQ_ACK_RETRY(s.reqStat(pkt_req), s.ackStat(pkt_ack, status), tryouts_prop.rvalue());
		if (success) {
			if (status.error & HerkulexServo::Status::ERROR_MASK) {
				std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << statusToString(status) << std::endl;
			}
		}
		else {
			log(Error) << "Unable query status of " << servo << " servo." << endlog();
			std::cout << servo << " ID = " << std::hex << s.getID() << std::dec << " QUERY ERROR" << std::endl;
		}
	}
}

void HerkulexArray::printRegisterRAM(const std::string& servo, const std::string& reg)
{
	unsigned long result = getRegisterRAM(servo, reg);
	std::cout << servo << " ID = " << std::hex << s.getID();
	if (result != READ_ERROR) {
		std::cout << reg << " = " << result << std::dec << std::endl;
	}
	else {
		std::cout << std::dec << " QUERY ERROR" << std::endl;
	}
}


void HerkulexArray::printAllRegistersRAM(const std::string& servo) 
{
	const HerkulexServo& s(servos.getServo(servo));
	unsigned int val;
	HerkulexServo::Status status;

	std::cout << servo << " ID = " << std::hex << s.getID(); << endl;
	for(auto r = s.register_mapper.registers.begin(); r != s.register_mapper.registers.end(); r++) {
		if (r->ram_addr != -1) {
			if (CMD_REQ_ACK_RETRY(s.reqRead_ram(pkt_req, r->name), s.ackRead_ram(pkt_ack, r->name, val, status), tryouts_prop.rvalue())) {
				std::cout << r->name << " = " << val << std::endl;
			}
			else {
				std::cout << r->name << " QUERY ERROR" << std::endl;
			}
			std::cout << std::dec << std::endl;
		}
	}
}

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

bool HerkulexArray::reqPosVel(HerkulexPacket& req, const std::string servo)
{
	if (!this->isConfigured()) return false;
	servos.getServo(servo).reqPosVel(req);
}

bool HerkulexArray::ackPosVel(HerkulexPacket& ack, const std::string servo, double& pos, double& vel) 
{
	if (!this->isConfigured()) return false;
	HerkulexServo::Status status;
	return servos.getServo(servo).ackPosVel(ack, servo, pos, vel, status);
}

void receivePacketCM(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt) 
{
	ack_buffer.Push(pkt);
	ack_mutex.lock();
	ack_cond.broadcast();
	ack_mutex.unlock();
}


//cmdSync(pkt_req, boost::bind(&HerkulexServo::ackRead_ram, &s, _1, boost::cref(r->name)));

bool sendRequest(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& req, boost::function<bool(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket&)> parse_ack)
{
	sweetie_bot_hardware_herkulex_msgs::HerkulexPacket * pkt_ack;
	unsigned int tryouts = tryouts_prop.rvalue();
	do {
		timeout_timer.arm(0, timeout_prop.rvalue());	
		cmdReqCP(req);
		tryouts--;
		do {
			ack_mutex.lock();
			while(ack_buffer.empty() && timeout_timer.isArmed(0) && !break_loop_flag ) ack_cond.wait(ack_mutex);
			ack_mutex.unlock();
			pkt_ack = ack_buffer.PopWithoutRelease();
			success = parse_ack(*pkt_ack);
			ack_buffer.Release(pkt_ack);
		} 
		while (!success && timeout_timer.isArmed() && !break_loop_flag);
		timeout_timer.killTimer(0);
	}
	while (!success && tryouts && !break_loop_flag);
	return success;
}	



		


bool cmdSync(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& req, sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& ack)
{
	//TODO 
	cmdReqCP(req);
	ack_mutex.lock();
	while(ack_buffer.empty()) ack_cond.wait(ack_mutex);
	ack_buffer.Pop(ack);
	ack_mutex.unlock();
}

bool HerkulexArray::startHook()
{
}

void HerkulexArray::updateHook()
{
}

void HerkulexArray::stopHook() 
{
}

void HerkulexArray::cleanupHook() {
	servos.clear();
	servos_init.clear();
	joints.clear();
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
