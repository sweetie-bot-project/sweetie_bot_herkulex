#include "herkulex_sched.hpp"

#include <rtt/Component.hpp>


using namespace sweetie_bot;
using namespace RTT;

namespace herkulex {

//Convinence macro fo logging.
std::ostream& resetfmt(std::ostream& s) {
	s.copyfmt(std::ios(NULL)); 
	return s;
}

HerkulexSched::HerkulexSched(std::string const& name) : 
	TaskContext(name, PreOperational),
	receivePacketCM("receivePacketCM", this->engine()),
	sendPacketDL("sendPacketDL", this->engine()),
	waitSendPacketDL("waitSendPacketDL", this->engine()),
	reqIJOG("reqIJOG"),
	reqPosVel("reqPosVel"),
	ackPosVel("ackPosVel"),
	reqPosVelExtended("reqPosVelExtended"),
	ackPosVelExtended("ackPosVelExtended"),
	cm_req_buffer(10, HerkulexPacket(), true),
	ack_buffer(10, HerkulexPacket(), true),
	timer(this),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("HerkulexSched");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
		this->fatal();
		return;
	}

	// INITIALIZATION
	// Check timer thread.
	if (!timer.getActivity() || !timer.getActivity()->thread()) {
		log(ERROR) << "Unable to start timer thread.";
		this->fatal();
		return;
	}
#ifdef SCHED_STATISTICS
	time_service = RTT::os::TimeService::Instance();
	if (time_service == nullptr) {
		log(ERROR) << "Unable to acquare TimeService.";
		this->fatal();
		return;
	}
#endif /* SCHED_STATISTICS */

	// Ports
	this->addEventPort("sync", sync_port)
		.doc("Timer syncronization event. This event indicates start of real time exchange round.");
	this->addPort("in_goals", goals_port)
		.doc("Servo goals.");
	this->addPort("out_joints", joints_port)
		.doc("Position and speed of servos from poll list.");
	this->addPort("out_states", states_port)
		.doc("State of servos from poll list. The level of verbosity depends on detailed_state property."); 
#ifdef SCHED_STATISTICS
	this->addPort("statistics", statistics_port)
		.doc("Real time exchange statistics."); 
#endif

	// Properties
	this->addProperty("period_RT_JOG", period_RT_JOG)
		.doc("Duration of realtime JOG round. During this round the component sends set goal command.")
		.set(0.01);
	this->addProperty("period_RT_read", period_RT_read)
		.doc("Duration of realtime exchange round. During this round the component sends queries to polled servo.")
		.set(0.04);
	this->addProperty("period_CM", period_CM)
		.doc("Duration of configuration and monitoring exchange round. During this round the component forwards request from `sendPacketCM` opertion to data link layer.")
		.set(0.01);
	this->addProperty("detailed_state", detailed_state)
		.doc("Request more detailed state from servos and publish it via `states` port.")
		.set(false);
	this->addProperty("poll_list", poll_list)
		.doc("List of servos, which state is read during real-time exchange round.");
	this->addProperty("poll_round_size", poll_round_size)
		.doc("Maximal number of servos to be polled during RT_READ round. Negative value means attempt to poll all servos.")
		.set(-1);
	this->addProperty("timeout", req_timeout)
		.doc("Servo request timeout (sec).")
		.set(0.005);
	this->addProperty("req_period", req_period)
		.doc("Servo request period (sec).")
		.set(0.005);
	
	// OPERATIONS: DATA LINK INTERFACE
	this->addOperation("receivePacketDL", &HerkulexSched::receivePacketDL, this, ClientThread) 
		.doc("Servo responce hook operation.") 
		.arg("pkt", "Received HerkulexPacket.");
	this->requires()->addOperationCaller(sendPacketDL);
	this->requires()->addOperationCaller(waitSendPacketDL);

	// OPERATIONS: CONFIGURATION AND MONITORING INTERFACE
	this->addOperation("sendPacketCM", &HerkulexSched::sendPacketCM, this, ClientThread) 
		.doc("Forward packet to data link interface during configuration and monitoring round.") 
		.arg("pkt", "HerkulexPacket to send.");
	this->requires()->addOperationCaller(receivePacketCM);

	// Protocol
	this->requires("protocol")->addOperationCaller(reqIJOG);
	this->requires("protocol")->addOperationCaller(reqPosVel);
	this->requires("protocol")->addOperationCaller(ackPosVel);
	this->requires("protocol")->addOperationCaller(reqPosVelExtended);
	this->requires("protocol")->addOperationCaller(ackPosVelExtended);
}

bool HerkulexSched::configureHook()
{
	// check if data link layer is ready
	if (! sendPacketDL.ready()) {
		log(ERROR) << "sendPacketDL opertions is not ready." << endlog(); 
		return false;
	}
	// check if protocol service presents
	if (!this->requires("protocol")->ready()) {
		log(ERROR) << "protocol service is not ready." << endlog(); 
		return false;
	}
	// reserve packet buffers
	req_pkt.data.resize(HerkulexPacket::DATA_SIZE);
	ack_buffer.data_sample(req_pkt);
	cm_req_buffer.data_sample(req_pkt);

	// reserve port buffers
	joints.name.resize(poll_list.size());
	joints.position.resize(poll_list.size());
	joints.velocity.resize(poll_list.size());

	states.name.resize(poll_list.size());
	states.pos.resize(poll_list.size());
	states.vel.resize(poll_list.size());
	states.status_error.resize(poll_list.size());
	states.status_detail.resize(poll_list.size());

	if (detailed_state) {
		states.pwm.resize(poll_list.size());
		states.pos_goal.resize(poll_list.size());
		states.pos_desired.resize(poll_list.size());
		states.vel_desired.resize(poll_list.size());
	}

	// set data samples
	joints_port.setDataSample(joints);
	states_port.setDataSample(states);

	log(INFO) << "HerkulexSched is configured!" << endlog(); 
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
	states.status_error.clear();
	states.status_detail.clear();
	states.not_responding.clear();

	//if (detailed_state) {
	states.pwm.clear();
	states.pos_goal.clear();
	states.pos_desired.clear();
	states.vel_desired.clear();
	//}
#ifdef SCHED_STATISTICS
	// reset statistics frame
	statistics.rt_jog_send_duration = 0;
	statistics.rt_read_start_time = 0;
	statistics.rt_read_req_duration1 = 0;
	statistics.rt_read_req_durationN = 0;
	statistics.rt_read_n_successes = 0;
	statistics.rt_read_n_errors = 0;
	statistics.last_erroneous_status = 0;
	statistics.cm_start_time = 0;
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
	// sart polling from begining of poll list
	poll_index = 0;

	// start timer
	if (!timer.getActivity()->thread()->start()) {
		log(ERROR) << "Unable to start timer." << endlog(); 
		return false;
	}
	log(INFO) << "HerkulexSched is started!" << endlog(); 
	return true;
}

void HerkulexSched::forwardAckPackagesToCM() 
{
	while (true) {
		HerkulexPacket * ack_pkt = ack_buffer.PopWithoutRelease();
		if (ack_pkt == nullptr) break; // ack_buffer is empty
		if (receivePacketCM.ready()) {
			receivePacketCM(*ack_pkt);
		}
		ack_buffer.Release(ack_pkt);
	}
}

void HerkulexSched::checkRTAckPackages()
{
	// check if ack buffer contains responces to RT requests
	// any other valid packages are rerouted to CM level
	
	while (true) {
		HerkulexPacket * ack_pkt = ack_buffer.PopWithoutRelease();
		if (ack_pkt == nullptr) break; // ack_buffer is empty

		double pos, vel;
		servo::Status status;
		bool success = false;
		// check if received packet is responce to one of previous requests
		// the front of the queue contains the oldest requests
		auto poll_index_it = poll_index_ack_queue.begin();
		for(; poll_index_it !=  poll_index_ack_queue.end(); poll_index_it++) {
			if (! detailed_state) {
				success = ackPosVel(*ack_pkt, poll_list[*poll_index_it], pos, vel, status);
			}
			else {
				success = ackPosVelExtended(*ack_pkt, poll_list[*poll_index_it], states, status) ;
			}
			if (success) break;
		}

		if (success) {
			// now save request results
			if (!detailed_state) {
				states.name.push_back(poll_list[*poll_index_it]);
				states.pos.push_back(pos);
				states.vel.push_back(vel);
				states.status_error.push_back(status.error);
				states.status_detail.push_back(status.detail);
			}
			else {
				pos = states.pos.back();
				vel = states.vel.back();
			}
			joints.name.push_back(poll_list[*poll_index_it]);
			joints.position.push_back(pos);
			joints.velocity.push_back(vel);

#ifdef SCHED_STATISTICS
			statistics.rt_read_n_errors += poll_index_it - poll_index_ack_queue.begin();
			statistics.rt_read_n_successes++;
			statistics.rt_read_req_durationN = timer.timeRemaining(REQUEST_TIMEOUT_TIMER);
			if (statistics.rt_read_n_successes + statistics.rt_read_n_errors == 0) {
				//statistics.rt_read_req_duration1 = time_service->secondsSince(statistics_sync_timestamp) - statistics.rt_read_start_time;
				statistics.rt_read_req_duration1 = statistics.rt_read_req_durationN;
			}
			if (status != 0) statistics.last_erroneous_status = status;
#endif /* SCHED_STATISTICS */

			// pop out of queue current reqest and all which preceeds it
			poll_index_ack_queue.dequeue(poll_index_it);

			if (log(DEBUG)) {
				log() << "RT ACK packet: servo_id: " << (int) ack_pkt->servo_id << " cmd: " << (int) ack_pkt->command << " data(" << ack_pkt->data.size() << ") ";
				log() << "pos = " << pos << " vel = " << vel << " ack_wait_queue_size = " << poll_index_ack_queue.size() << endlog();
			}
		}
		else {
			// packet is not RT request responce. Reroute it to CM layer.
			if (receivePacketCM.ready()) {
				receivePacketCM(*ack_pkt);
			}

			if (log(DEBUG)) {
				log() << std::dec << std::setw(2) << std::setfill('0');
				log() << "Unexpected ACK packet: servo_id: " << (int) ack_pkt->servo_id << " cmd: " << (int) ack_pkt->command << " data(" << ack_pkt->data.size() << "): ";
				for(auto c = ack_pkt->data.begin(); c != ack_pkt->data.end(); c++) log() << (int) *c << " ";
				log() << "ack_wait_queue_size = " << poll_index_ack_queue.size() << endlog();
			}
		}
		// release packet
		ack_buffer.Release(ack_pkt);
	}
}

void HerkulexSched::updateHook()
{
	bool success;
	SchedTimer::TimerId timer_id;

	if (log(DEBUG)) {
		log() << "updateHook: sched_state = " << sched_state 
			<< " ack_quuue_size = " << poll_index_ack_queue.size()
			<< " period_timer = " << timer.timeRemaining(REQUEST_PERIOD_TIMER) 
			<< " timeout_timer = " << timer.timeRemaining(REQUEST_TIMEOUT_TIMER) 
			<< " round_timer = " << timer.timeRemaining(ROUND_TIMER) << endlog();
	}
	
	switch (sched_state) {
		case SEND_JOG:
			// wait sync and send JOG command
			if (sync_port.read(timer_id) == NewData) {
				timer.arm(ROUND_TIMER, this->period_RT_JOG);
#ifdef SCHED_STATISTICS
				statistics_sync_timestamp = time_service->getTicks();
#endif /* SCHED_STATISTICS */

				goals_port.read(goals, false);

				if (goals.name.size() != goals.target_pos.size() || goals.name.size() != goals.playtime.size()) {
					log(WARN) << "Goal message has incorrect structure." << endlog();
				}
				else {
					reqIJOG(req_pkt, goals);
					sendPacketDL(req_pkt);

					if (log(DEBUG)) {
						log() << "Start RT round." << std::endl;
						log() << std::dec << std::setw(2) << std::setfill('0');
						log() << "REQ packet: servo_id: "  << (int) req_pkt.servo_id << " cmd: " << (int) req_pkt.command << " data(" << req_pkt.data.size() << "): ";
						for(auto c = req_pkt.data.begin(); c != req_pkt.data.end(); c++) log() << (int) *c << " ";
						log() << resetfmt << endlog();
					}
				}
				// reset servo poll variables
				if (poll_round_size < 0) { // attempt poll all servos in one round
					poll_index = 0;
					poll_end_index = poll_list.size();
				}
				else { // poll only poll_round_size servos in current round and preserve poll_index
					if (poll_index >= poll_list.size()) poll_index = 0; // end of poll list
					poll_end_index = poll_index + poll_round_size;
					if (poll_end_index > poll_list.size()) poll_end_index = poll_list.size();
				}
				poll_index_ack_queue.clear();
				clearPortBuffers();

				if (! waitSendPacketDL.ready()) {
					waitSendPacketDL();
				}
#ifdef SCHED_STATISTICS
				statistics.rt_jog_send_duration = time_service->secondsSince(statistics_sync_timestamp);
#endif /* SCHED_STATISTICS */

				sched_state = SEND_JOG_WAIT;
				break;
			}
			// Forward all incoming messages to CM interface while waiting sync or before start polling.
			forwardAckPackagesToCM();
			break;

		case SEND_JOG_WAIT:
			// wait for round timer expires
			if (timer.isArmed(ROUND_TIMER)) {
				break;
			}

			sched_state = SEND_READ_REQ;
			timer.arm(ROUND_TIMER, period_RT_read);
#ifdef SCHED_STATISTICS
				statistics.rt_read_start_time = time_service->secondsSince(statistics_sync_timestamp);
#endif /* SCHED_STATISTICS */

		case SEND_READ_REQ:
			// in this state scheduler sends RT read requests

			if (poll_index >= poll_end_index || !timer.isArmed(ROUND_TIMER)) {
				// switch to next state:
				// there are no more requests to send or RT round is finised
				timer.killTimer(ROUND_TIMER);

				// waiting for the responce of the last servo 
				sched_state = WAIT_READ_ACK;
				this->trigger();
				break;
			}
			if (!timer.isArmed(REQUEST_PERIOD_TIMER) || poll_index_ack_queue.empty()) {
				// send next RT read request:
				// request period timer fired or all ACK packages is received

				// form request package
				if (! detailed_state) {
					success = reqPosVel(req_pkt, poll_list[poll_index]);
				}
				else {
					success = reqPosVelExtended(req_pkt, poll_list[poll_index]);
				}
				if (!success) {
					// skip servo
					poll_index++;
					this->trigger();
					break;
				}
				// poll index of coresponding servo to ack wait list
				poll_index_ack_queue.enqueue(poll_index);
				// switch to next servo
				poll_index++;
	
				timer.arm(REQUEST_PERIOD_TIMER, this->req_period);
				timer.arm(REQUEST_TIMEOUT_TIMER, this->req_timeout);
				sendPacketDL(req_pkt);

				if (log(DEBUG)) {
					log() << std::dec << std::setw(2) << std::setfill('0');
					log() << "RT REQ packet: servo_id: "  << (int) req_pkt.servo_id << " cmd: " << (int) req_pkt.command << " data(" << req_pkt.data.size() << "): ";
					for(auto c = req_pkt.data.begin(); c != req_pkt.data.end(); c++) log() << (int) *c << " ";
					log() << resetfmt << endlog();
				}
			}

			// check is any ACK packet is received
			checkRTAckPackages();

			if (poll_index_ack_queue.empty()) {
				// no more packets to wait: procced to next request
				this->trigger();
			}
			break;
		
		case WAIT_READ_ACK:
			// RT round is almost finished: scheduler waits for the response of the last servo.
			if (!poll_index_ack_queue.empty() && timer.isArmed(REQUEST_TIMEOUT_TIMER)) {
				// check if we have received RT package
				checkRTAckPackages();
			}
			if (poll_index_ack_queue.empty() || !timer.isArmed(REQUEST_TIMEOUT_TIMER)) {
				// no more packets to wait or request timeout is expired
				// publish joint states and statistics

				ros::Time timestamp = ros::Time::now();
				joints.header.stamp = timestamp;
				states.header.stamp = timestamp;

				joints_port.write(joints);
				states_port.write(states);

#ifdef SCHED_STATISTICS
				statistics.cm_start_time = time_service->secondsSince(statistics_sync_timestamp);
				statistics.rt_read_n_errors += poll_index_ack_queue.size();
				statistics_port.write(statistics);
#endif /* SCHED_STATISTICS */
				
				// clear wait queue
				poll_index_ack_queue.clear();	

				timer.arm(ROUND_TIMER, this->period_CM);
				sched_state = CM_ROUND;
				log(DEBUG) << "Start CM round." << endlog();

				this->trigger();
				break;
			}
			break;

		case CM_ROUND:
			if (!timer.isArmed(ROUND_TIMER)) {
				SchedTimer::TimerId timer_id;
				if (sync_port.read(timer_id) == NewData) {
					// we get sync msg before timer expires
					log(ERROR) << "sync message is received before scheduler rounds have been finished." << endlog();
					// now wait for next sync
					// TODO: display statistics
				}
				sched_state = SEND_JOG;
				break;
			}

			// process requests from CM layer
			{
				HerkulexPacket * cm_req_pkt = cm_req_buffer.PopWithoutRelease();
				if (cm_req_pkt != nullptr) {  // cm_req_buffer is not empty
					sendPacketDL(*cm_req_pkt);
					cm_req_buffer.Release(cm_req_pkt);
				}
				if (!cm_req_buffer.empty()) this->trigger();
			}
			// foward received packets to CM layer
			forwardAckPackagesToCM();

			break;
	} // case (sched_state)
}
				


void HerkulexSched::receivePacketDL(const HerkulexPacket& pkt) 
{
	if (this->isRunning()) {
		// buffer message to updateHook processing
		ack_buffer.Push(pkt);
		this->trigger();
	}
	else {
		// forward message to CM subsystem
		if (receivePacketCM.ready()) {
			log(DEBUG) << "Forward packet to CM subsytem." << endlog();

			receivePacketCM(pkt);
		}
	}
}

void HerkulexSched::sendPacketCM(const HerkulexPacket& pkt) 
{
	if (this->isRunning()) {
		// buffer message to updateHook processing
		cm_req_buffer.Push(pkt);
		this->trigger();
	}
	else {
		// forward message to data link layer
		if (sendPacketDL.ready()) { 
			log(DEBUG) << "Forward packet to DL layer" << endlog();

			sendPacketDL(pkt);
		}
		else {
			log(ERROR) << "Data link layer (sendPacketDL operation) is not ready." << endlog();
		}
	}
}

void HerkulexSched::stopHook() 
{
	log(INFO) << "HerkulexSched is stopped!" << endlog(); 
}

void HerkulexSched::cleanupHook() 
{
}

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
ORO_CREATE_COMPONENT(herkulex::HerkulexSched)
