#include "herkulex_sched.hpp"

#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

//TODO Where should be this code?
std::ostream& resetfmt(std::ostream& s) {
	s.copyfmt(std::ios(NULL)); 
	return s;
}

using namespace RTT;

HerkulexSched::HerkulexSched(std::string const& name) : 
	TaskContext(name, PreOperational),
	receivePacketCM("receivePacketCM"),
	sendPacketDL("sendPacketDL"),
	reqIJOG("reqIJOG"),
	reqPosVel("reqPosVel"),
	ackPosVel("ackPosVel"),
	reqState("reqState"),
	ackState("ackState"),
	cm_req_buffer(10, HerkulexPacket(), true),
	ack_buffer(10, HerkulexPacket(), true),
	timer(*this)
{
	// Ports
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event indicates start of real time exchange round.");
	this->addPort("goals", goals_port)
		.doc("Servo goals.");
	this->addPort("joints", joints_port)
		.doc("Position and speed of servos from poll list.");
	this->addPort("states", states_port)
		.doc("State of servos from poll list. The level of verbosity depends on detailed_state property."); 
#ifdef SCHED_STATISTICS
	this->addPort("statistics", statistics_port)
		.doc("Real time exchange statistics."); 
#endif

	// Properties
	this->addProperty("period_RT", period_RT)
		.doc("Duration of realtime exchange round. During this round the component sends set goal command and reads` states of polled servo.")
		.set(0.04);
	this->addProperty("period_CM", period_CM)
		.doc("Duration of configuration and monitoring exchange round. During this round the component forwards request from `sendPacketCM` opertion to data link layer.")
		.set(0.01);
	this->addProperty("detailed_state", detailed_state)
		.doc("Request more detailed state from servos and publish it via `states` port.")
		.set(false);
	this->addProperty("poll_list", poll_list)
		.doc("List of servos, which state is read during real-time exchange round.");
	this->addProperty("timeout", timeout)
		.doc("Servo request timeout (sec).")
		.set(0.005);
	
	// OPERATIONS: DATA LINK INTERFACE
	this->addOperation("receivePacketDL", &HerkulexSched::receivePacketDL, this, ClientThread) 
		.doc("Servo responce hook operation.") 
		.arg("pkt", "Received HerkulexPacket.");
	this->requires()->addOperationCaller(sendPacketDL);

	// OPERATIONS: CONFIGURATION AND MONITORING INTERFACE
	this->addOperation("sendPacketCM", &HerkulexSched::receivePacketDL, this, ClientThread) 
		.doc("Forward packet to data link interface during configuration and monitoring round.") 
		.arg("pkt", "HerkulexPacket to send.");
	this->requires()->addOperationCaller(receivePacketCM);

	// Protocol
	this->requires("protocol")->addOperationCaller(reqIJOG);
	this->requires("protocol")->addOperationCaller(reqPosVel);
	this->requires("protocol")->addOperationCaller(ackPosVel);
	this->requires("protocol")->addOperationCaller(reqState);
	this->requires("protocol")->addOperationCaller(ackState);
}

bool HerkulexSched::configureHook()
{
	Logger::In("HerkulexSched");
	// check if data link layer is ready
	if (! sendPacketDL.ready()) {
		log(Error) << "sendPacketDL opertion is not ready." << endlog(); 
		return false;
	}
	// check if protocol service presents
	if (!this->requires("protocol")->ready()) {
		log(Error) << "protocol service is not ready." << endlog(); 
		return false;
	}
	// reserve packet buffers
	req_pkt.data.resize(HerkulexPacket::DATA_SIZE);
	ack_buffer.data_sample(req_pkt);
	cm_req_buffer.data_sample(req_pkt);

	// reserve port buffers
	joints.name.reserve(poll_list.size());
	joints.position.reserve(poll_list.size());
	joints.velocity.reserve(poll_list.size());

	states.name.reserve(poll_list.size());
	states.pos.reserve(poll_list.size());
	states.vel.reserve(poll_list.size());
	states.status.reserve(poll_list.size());

	if (detailed_state) {
		states.pwm.reserve(poll_list.size());
		states.pos_goal.reserve(poll_list.size());
		states.pos_desired.reserve(poll_list.size());
		states.vel_desired.reserve(poll_list.size());
	}

	// set data samples
	joints_port.setDataSample(joints);
	states_port.setDataSample(states);

	return true;
}

void HerkulexSched::clearPortBuffers() {
	joints.name.clear();
	joints.position.clear();
	joints.velocity.clear();
	joints.effort.clear();

	states.name.clear();
	states.pos.clear();
	states.vel.clear();
	states.status.clear();

	//if (detailed_state) {
	states.pwm.clear();
	states.pos_goal.clear();
	states.pos_desired.clear();
	states.vel_desired.clear();
	//}
#ifdef SCHED_STATISTICS
	// reset statistics
	statistics.avg_request_duration = 0;
	statistics.rt_round_duration = 0;
	statistics.n_success = 0;
	statistics.n_errors = 0;
	statistics.last_erroneous_status = 0;
#endif /* SCHED_STATISTICS */
}

bool HerkulexSched::startHook()
{
	sched_state = SEND_JOG;

	clearPortBuffers();

	// prevent triggering by buffered message
	SchedTimer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// get input port sample
	goals_port.getDataSample(goals);

	return true;
}

void HerkulexSched::updateHook()
{
	bool success;
	SchedTimer::TimerId timer_id;
	
	switch (sched_state) {
		case SEND_JOG:
			// wait sync and send JOG command
			if (sync_port.read(timer_id) == NewData) {
				timer.arm(ROUND_TIMER, this->period_RT);
				
				goals_port.read(goals, false);

				if (goals.name.size() != goals.target_pos.size() || goals.name.size() != goals.playtime.size()) {
					Logger::In("HerkulexSched");
					log(Warning) << "goal message has incorrect structure." << endlog();
				}
				else {
					reqIJOG(req_pkt, goals);
					sendPacketDL(req_pkt);

					if (log().getLogLevel() >= Logger::Debug) {
						Logger::In("HerkulexSched");
						log(Debug) << "Start RT round." << endlog();
						log() << Logger::Debug << std::hex << std::setw(2) << std::setfill('0');
						log() << "REQ packet: servo_id: "  << (int) req_pkt.servo_id << " cmd: " << (int) req_pkt.command << " data(" << req_pkt.data.size() << "): ";
						for(auto c = req_pkt.data.begin(); c != req_pkt.data.end(); c++) log() << (int) *c << " ";
						log() << resetfmt << endlog();
					}
				}

				sched_state = SEND_READ_REQ;
				// reset servo poll variables
				poll_list_index = 0;
				clearPortBuffers();
				this->getActivity()->trigger();
			}
			break;

		case SEND_READ_REQ:
			if (poll_list_index >= poll_list.size() || !timer.isArmed(ROUND_TIMER)) {
				// RT round is finished
#ifdef SCHED_STATISTICS
				statistics.rt_round_duration = period_RT - timer.timeRemaining(ROUND_TIMER);
#endif /* SCHED_STATISTICS */

				timer.killTimer(ROUND_TIMER);

				joints_port.write(joints);
#ifdef SCHED_STATISTICS
				statistics_port.write(statistics);
#endif /* SCHED_STATISTICS */

				// start configuration and monitoring round
				sched_state = CM_ROUND;
				timer.arm(ROUND_TIMER, period_CM);
				if (log().getLogLevel() >= Logger::Debug) {
					Logger::In("HerkulexSched");
					log(Debug) << "Start CM round." << endlog();
				}
				this->getActivity()->trigger();
				break;
			}
			// form request to servo
			if (! detailed_state) {
				success = reqPosVel(req_pkt, poll_list[poll_list_index]);
			}
			else {
				success = reqState(req_pkt, poll_list[poll_list_index]);
			}
			if (!success) {
				// skip servo
				poll_list_index++;
				this->getActivity()->trigger();
				break;
			}
			
			timer.arm(REQUEST_TIMEOUT_TIMER, this->timeout);
			sendPacketDL(req_pkt);
			if (log().getLogLevel() >= Logger::Debug) {
				Logger::In("HerkulexSched");
				log() << Logger::Debug << std::hex << std::setw(2) << std::setfill('0');
				log() << "REQ packet: servo_id: "  << (int) req_pkt.servo_id << " cmd: " << (int) req_pkt.command << " data(" << req_pkt.data.size() << "): ";
				for(auto c = req_pkt.data.begin(); c != req_pkt.data.end(); c++) log() << (int) *c << " ";
				log() << resetfmt << endlog();
			}

			sched_state = RECEIVE_READ_ACK;
			break;
			
		case RECEIVE_READ_ACK:
			if (!timer.isArmed(REQUEST_TIMEOUT_TIMER)) {
				sched_state = SEND_READ_REQ;
#ifdef SCHED_STATISTICS
				statistics.n_errors++;
#endif /* SCHED_STATISTICS */
				if (log().getLogLevel() >= Logger::Debug) {
					Logger::In("HerkulexSched");
					log(Debug) << "ACK timeout" << endlog();
				}
				this->getActivity()->trigger();
				break;
			}

			while (!ack_buffer.empty()) {
				HerkulexPacket * ack_pkt = ack_buffer.PopWithoutRelease();
				double pos, vel;
				unsigned int status;
				if (! detailed_state) {
					success = ackPosVel(*ack_pkt, poll_list[poll_list_index], pos, vel, status);
				}
				else {
					success = ackState(*ack_pkt, poll_list[poll_list_index], states, status) ;
					pos = states.pos.back();
					vel = states.vel.back();
				}
				if (success && log().getLogLevel() >= Logger::Debug) {
					Logger::In("HerkulexSched");
					log() << Logger::Debug << std::hex << std::setw(2) << std::setfill('0');
					log() << "ACK packet: servo_id: " << (int) ack_pkt->servo_id << " cmd: " << (int) ack_pkt->command << " data(" << ack_pkt->data.size() << "): ";
					for(auto c = ack_pkt->data.begin(); c != ack_pkt->data.end(); c++) log() << (int) *c << " ";
					log() << resetfmt << endlog();
				}
				ack_buffer.Release(ack_pkt);

				if (success) {
					// save request results
					joints.name.push_back(poll_list[poll_list_index]);
					joints.position.push_back(pos);
					joints.velocity.push_back(vel);
					if (!detailed_state) {
						states.name.push_back(poll_list[poll_list_index]);
						states.pos.push_back(pos);
						states.vel.push_back(vel);
						states.status.push_back(status);
					}
#ifdef SCHED_STATISTICS
					statistics.avg_request_duration = (statistics.n_success * statistics.avg_request_duration + timeout - timer.timeRemaining(REQUEST_TIMEOUT_TIMER)) / (statistics.n_success + 1);
					statistics.n_success++;
#endif /* SCHED_STATISTICS */
					timer.killTimer(REQUEST_TIMEOUT_TIMER);
					poll_list_index++;
					sched_state = SEND_READ_REQ;
					this->getActivity()->trigger();
					break;
				}
#ifdef SCHED_STATISTICS
				else {
					statistics.last_erroneous_status = status;
				}
#endif /* SCHED_STATISTICS */
			}
			break;

		case CM_ROUND:
			if (!timer.isArmed(ROUND_TIMER)) {
				SchedTimer::TimerId timer_id;
				if (sync_port.read(timer_id) == NewData) {
					// we get sync msg before timer espires
					Logger::In("HerkulexSched");
					log(Error) << "sync message is received before scheduler rounds have been finished." << endlog();
					this->exception();
					break;
				}
				sched_state = SEND_JOG;
				break;
			}

			// send and receive packets completely asyncronically
			while (!ack_buffer.empty()) {
				HerkulexPacket * cm_ack_pkt = cm_req_buffer.PopWithoutRelease();
				if (receivePacketCM.ready()) {
					receivePacketCM(*cm_ack_pkt);
				}
				ack_buffer.Release(cm_ack_pkt);
			}
			if (!cm_req_buffer.empty()) {
				HerkulexPacket * cm_req_pkt = cm_req_buffer.PopWithoutRelease();
				sendPacketDL(*cm_req_pkt);
				cm_req_buffer.Release(cm_req_pkt);
				if (!cm_req_buffer.empty()) this->getActivity()->trigger();
			}
			break;

	} // case (sched_state)
}
				

void HerkulexSched::receivePacketDL(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt) 
{
	if (this->isRunning()) {
		// buffer message to updateHook processing
		ack_buffer.Push(pkt);
		this->getActivity()->trigger();
	}
	else {
		// forward message to CM subsystem
		if (receivePacketCM.ready()) {
			receivePacketCM(pkt);
		}
	}
}

void HerkulexSched::sendPacketCM(const sweetie_bot_hardware_herkulex_msgs::HerkulexPacket& pkt) 
{
	if (this->isRunning()) {
		// buffer message to updateHook processing
		cm_req_buffer.Push(pkt);
		this->getActivity()->trigger();
	}
	else {
		// forward message to data link layer
		if (sendPacketDL.ready()) { 
			sendPacketDL(pkt);
		}
		else {
			Logger::In("HerkulexSched");
			log(Error) << "Data link layer sendPacketDL operation is not ready." << endlog();
		}
	}
}

void HerkulexSched::stopHook() 
{
}

void HerkulexSched::cleanupHook() 
{
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HerkulexSched)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(HerkulexSched)
