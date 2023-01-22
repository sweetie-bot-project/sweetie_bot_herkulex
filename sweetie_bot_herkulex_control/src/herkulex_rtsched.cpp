#include "herkulex_rtsched.hpp"

#include <rtt/Component.hpp>


using namespace sweetie_bot;
using namespace RTT;

namespace herkulex {

//Convinence macro fo logging.
std::ostream& resetfmt(std::ostream& s) {
	s.copyfmt(std::ios(NULL)); 
	return s;
}

HerkulexRTSched::HerkulexRTSched(std::string const& name) : 
	TaskContext(name, PreOperational),
	receivePacketCM("receivePacketCM", this->engine()),
	sendPacketDL("sendPacketDL", this->engine()),
	waitSendPacketDL("waitSendPacketDL", this->engine()),
	reqRT_EXCHANGE("reqRT_EXCHANGE"),
	ackRT_EXCHANGE("ackRT_EXCHANGE"),
	cm_req_buffer(10, HerkulexPacket(), true),
	ack_buffer(10, HerkulexPacket(), true),
	timer(this),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("HerkulexRTSched");
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
	this->addPort("in_joints_ref", joints_ref_port)
		.doc("Servo reference  (position, speed, feedforward effort) or (effort).");
	this->addPort("out_joints", joints_port)
		.doc("Actual joint state: position, speed, effort.");
#ifdef SCHED_STATISTICS
	this->addPort("out_statistics", statistics_port)
		.doc("Real time exchange statistics."); 
#endif

	// Properties
	this->addProperty("period_RT", period_RT)
		.doc("Duration of realtime exchange round (sec). Should be long enough to perform RT_EXCHANGE command and receive servos response.")
		.set(0.01);
	this->addProperty("period_CM", period_CM)
		.doc("Duration of configuration and monitoring exchange round (sec). During this round the component forwards request from `sendPacketCM` opertion to data link layer.")
		.set(0.01);
	this->addProperty("timeout", req_timeout)
		.doc("Servo request timeout (sec).")
		.set(0.005);
	
	// OPERATIONS: DATA LINK INTERFACE
	this->addOperation("receivePacketDL", &HerkulexRTSched::receivePacketDL, this, ClientThread) 
		.doc("Servo responce hook operation.") 
		.arg("pkt", "Received HerkulexPacket.");
	this->requires()->addOperationCaller(sendPacketDL);
	this->requires()->addOperationCaller(waitSendPacketDL);

	// OPERATIONS: CONFIGURATION AND MONITORING INTERFACE
	this->addOperation("sendPacketCM", &HerkulexRTSched::sendPacketCM, this, ClientThread) 
		.doc("Forward packet to data link interface during configuration and monitoring round.") 
		.arg("pkt", "HerkulexPacket to send.");
	this->requires()->addOperationCaller(receivePacketCM);

	// Protocol
	this->requires("protocol")->addOperationCaller(reqRT_EXCHANGE);
	this->requires("protocol")->addOperationCaller(ackRT_EXCHANGE);
}

bool HerkulexRTSched::configureHook()
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

	// TODO: reserve port buffers
	// TODO: set data samples

	log(INFO) << "HerkulexRTSched is configured!" << endlog(); 
	return true;
}

void HerkulexRTSched::clearPortBuffers() {
	joints.name.clear();
	joints.position.clear();
	joints.velocity.clear();
	joints.effort.clear();

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

bool HerkulexRTSched::startHook()
{

	sched_state = RT_ROUND_REQ;

	clearPortBuffers();

	// prevent triggering by buffered message
	SchedTimer::TimerId timer_id;
	sync_port.readNewest(timer_id);
	// get input port sample
	joints_ref_port.getDataSample(joints);

	// start timer
	if (!timer.getActivity()->thread()->start()) {
		log(ERROR) << "Unable to start timer." << endlog(); 
		return false;
	}
	log(INFO) << "HerkulexRTSched is started!" << endlog(); 
	return true;
}

void HerkulexRTSched::forwardAckPackagesToCM() 
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

void HerkulexRTSched::checkRTAckPackages()
{
}

void HerkulexRTSched::updateHook()
{
	bool success;
	SchedTimer::TimerId timer_id;

	if (log(DEBUG)) {
		log() << "updateHook: sched_state = " << sched_state 
			<< " timeout_timer = " << timer.timeRemaining(REQUEST_TIMEOUT_TIMER) 
			<< " round_timer = " << timer.timeRemaining(ROUND_TIMER) << endlog();
	}
	
	switch (sched_state) {
		case RT_ROUND_REQ:
			// wait sync and send RT_EXCHANGE command
			if (sync_port.read(timer_id) == NewData) {
				timer.arm(ROUND_TIMER, this->period_RT);
#ifdef SCHED_STATISTICS
				statistics_sync_timestamp = time_service->getTicks();
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

				joints_ref_port.read(joints_ref, false);
				bool success = reqRT_EXCHANGE(req_pkt, joints_ref);

				if (success) {
					sendPacketDL(req_pkt);

					if (log(DEBUG)) {
						log() << "Start RT_EXCHANGE round." << std::endl;
						log() << "REQ packet: servo_id: "  << (int) req_pkt.servo_id << " cmd: " << (int) req_pkt.command << " data(" << req_pkt.data.size() << ") ";
						log() << resetfmt << endlog();
					}

					clearPortBuffers();

					if (! waitSendPacketDL.ready()) {
						waitSendPacketDL();
					}

#ifdef SCHED_STATISTICS
					statistics.rt_jog_send_duration = time_service->secondsSince(statistics_sync_timestamp);
#endif /* SCHED_STATISTICS */
				
					sched_state = RT_ROUND_ACK;
				}
				else {
					log(WARN) << "Unable to form RT_EXCHANGE request." << endlog();

					sched_state = CM_ROUND;
				}


				break;
			}
			break;
		
		case RT_ROUND_ACK:
			if (timer.isArmed(ROUND_TIMER)) {
				// check if ack buffer contains responces to RT requests
				// any other valid packages are rerouted to CM level
				while (true) {
					HerkulexPacket * ack_pkt = ack_buffer.PopWithoutRelease();
					if (ack_pkt == nullptr) break; // ack_buffer is empty

					servo::Status status;
					double temperature;
					bool success = ackRT_EXCHANGE(*ack_pkt, joints, temperature, status);

#ifdef SCHED_STATISTICS
					statistics.rt_read_req_durationN = timer.timeRemaining(ROUND_TIMER);
					if (statistics.rt_read_n_successes == 0) {
						statistics.rt_read_req_duration1 = statistics.rt_read_req_durationN;
					}
					statistics.rt_read_n_successes++;
					if (status.isErrorStatus()) statistics.last_erroneous_status = status;
#endif /* SCHED_STATISTICS */

					if (success) {
						// TODO: servo status monitoring
						
						if (log(DEBUG)) {
							log() << "RT ACK packet: servo_id: " << (int) ack_pkt->servo_id << " cmd: " << (int) ack_pkt->command << " data(" << ack_pkt->data.size() << ") ";
							log() << joints.name.back() <<  ": pos = " << joints.position.back() << " vel = " << joints.velocity.back() << " effort = " << joints.effort.back() << endlog();
						}
					}
					else {
						// packet is not RT request responce. Reroute it to CM layer.
						if (receivePacketCM.ready()) {
							receivePacketCM(*ack_pkt);
						}

						if (log(DEBUG)) {
							log() << "Unexpected ACK packet: servo_id: " << (int) ack_pkt->servo_id << " cmd: " << (int) ack_pkt->command << " data(" << ack_pkt->data.size() << ") ";
						}
					}
					// release packet
					ack_buffer.Release(ack_pkt);
				}
			}
			else {
				// publish joint states and statistics
				ros::Time timestamp = ros::Time::now();
				joints.header.stamp = timestamp;
				joints_port.write(joints);

#ifdef SCHED_STATISTICS
				statistics.cm_start_time = time_service->secondsSince(statistics_sync_timestamp);
				statistics.rt_read_n_errors = 0;
				statistics_port.write(statistics);
#endif /* SCHED_STATISTICS */

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
				sched_state = RT_ROUND_REQ;
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
				


void HerkulexRTSched::receivePacketDL(const HerkulexPacket& pkt) 
{
	if (this->isRunning()) {
		// buffer message to updateHook processing
		ack_buffer.Push(pkt);
		this->trigger();
	}
	else {
		// forward message to CM subsystem
		if (receivePacketCM.ready()) {
			log(DEBUG) << "Forward packet to CM layer." << endlog();

			receivePacketCM(pkt);
		}
	}
}

void HerkulexRTSched::sendPacketCM(const HerkulexPacket& pkt) 
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

void HerkulexRTSched::stopHook() 
{
	log(INFO) << "HerkulexRTSched is stopped!" << endlog(); 
}

void HerkulexRTSched::cleanupHook() 
{
}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HerkulexRTSched)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(herkulex::HerkulexRTSched)
