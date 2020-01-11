#include "herkulex_sched.hpp"

#include <rtt/Component.hpp>

using namespace sweetie_bot;
using namespace RTT;

namespace herkulex
{

// Convinence macro fo logging.
std::ostream& resetfmt(std::ostream& s)
{
  s.copyfmt(std::ios(NULL));
  return s;
}

HerkulexSched::HerkulexSched(std::string const& name)
    : TaskContext(name, PreOperational), receivePacketCM("receivePacketCM", this->engine()),
      sendPacketDL("sendPacketDL", this->engine()), waitSendPacketDL("waitSendPacketDL", this->engine()),
      reqIJOG("reqIJOG"), reqPosVel("reqPosVel"), ackPosVel("ackPosVel"), reqState("reqState"), ackState("ackState"),
      cm_req_buffer(10, HerkulexPacket(), true), ack_buffer(10, HerkulexPacket(), true), timer(this),
      log(logger::categoryFromComponentName(name))
{
  if (!log.ready())
  {
    RTT::Logger::In in("HerkulexSched");
    RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
    this->fatal();
    return;
  }

  // INITIALIZATION
  // Check timer thread.
  if (!timer.getActivity() || !timer.getActivity()->thread())
  {
    log(ERROR) << "Unable to start timer thread.";
    this->fatal();
    return;
  }
#ifdef SCHED_STATISTICS
  time_service = RTT::os::TimeService::Instance();
  if (time_service == nullptr)
  {
    log(ERROR) << "Unable to acquare TimeService.";
    this->fatal();
    return;
  }
#endif /* SCHED_STATISTICS */

  // Ports
  this->addEventPort("sync", sync_port)
      .doc("Timer syncronization event. This event indicates start of real time exchange round.");
  this->addPort("in_goals", goals_port).doc("Servo goals.");
  this->addPort("out_joints", joints_port).doc("Position and speed of servos from poll list.");
  this->addPort("out_states", states_port)
      .doc("State of servos from poll list. The level of verbosity depends on detailed_state property.");
#ifdef SCHED_STATISTICS
  this->addPort("statistics", statistics_port).doc("Real time exchange statistics.");
#endif

  // Properties
  this->addProperty("period_RT_JOG", period_RT_JOG)
      .doc("Duration of realtime JOG round. During this round the component sends set goal command.")
      .set(0.01);
  this->addProperty("period_RT_read", period_RT_read)
      .doc("Duration of realtime exchange round. During this round the component sends queries to polled servo.")
      .set(0.04);
  this->addProperty("period_CM", period_CM)
      .doc("Duration of configuration and monitoring exchange round. During this round the component forwards request "
           "from `sendPacketCM` opertion to data link layer.")
      .set(0.01);
  this->addProperty("detailed_state", detailed_state)
      .doc("Request more detailed state from servos and publish it via `states` port.")
      .set(false);
  this->addProperty("poll_list", poll_list).doc("List of servos, which state is read during real-time exchange round.");
  this->addProperty("poll_round_size", poll_round_size)
      .doc("Maximal number of servos to be polled during RT_READ round. Negative value means attempt to poll all "
           "servos.")
      .set(-1);
  this->addProperty("timeout", timeout).doc("Servo request timeout (sec).").set(0.005);

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
  this->requires("protocol")->addOperationCaller(reqState);
  this->requires("protocol")->addOperationCaller(ackState);
}

bool HerkulexSched::configureHook()
{
  // check if data link layer is ready
  if (!sendPacketDL.ready())
  {
    log(ERROR) << "sendPacketDL opertions is not ready." << endlog();
    return false;
  }
  // check if protocol service presents
  if (!this->requires("protocol")->ready())
  {
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
  states.error.resize(poll_list.size());
  states.detail.resize(poll_list.size());

  if (detailed_state)
  {
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

void HerkulexSched::clearPortBuffers()
{
  joints.name.clear();
  joints.position.clear();
  joints.velocity.clear();
  joints.effort.clear();

  states.name.clear();
  states.pos.clear();
  states.vel.clear();
  states.error.clear();
  states.detail.clear();

  // if (detailed_state) {
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
  if (!timer.getActivity()->thread()->start())
  {
    log(ERROR) << "Unable to start timer." << endlog();
    return false;
  }
  log(INFO) << "HerkulexSched is started!" << endlog();
  return true;
}

void HerkulexSched::updateHook()
{
  bool success;
  SchedTimer::TimerId timer_id;

  if (log(DEBUG))
  {
    log() << "updateHook: sched_state = " << sched_state
          << " timeout_timer = " << timer.timeRemaining(REQUEST_TIMEOUT_TIMER)
          << " round_timer = " << timer.timeRemaining(ROUND_TIMER) << endlog();
  }

  switch (sched_state)
  {
  case SEND_JOG:
    // wait sync and send JOG command
    if (sync_port.read(timer_id) == NewData)
    {
      timer.arm(ROUND_TIMER, this->period_RT_JOG);
#ifdef SCHED_STATISTICS
      statistics_sync_timestamp = time_service->getTicks();
#endif /* SCHED_STATISTICS */

      goals_port.read(goals, false);

      if (goals.name.size() != goals.target_pos.size() || goals.name.size() != goals.playtime.size())
      {
        log(WARN) << "Goal message has incorrect structure." << endlog();
      }
      else
      {
        reqIJOG(req_pkt, goals);
        // use send mode becuase of #1 bug
        sendPacketDL_handle = sendPacketDL.send(req_pkt);

        if (log(DEBUG))
        {
          log() << "Start RT round." << std::endl;
          log() << std::dec << std::setw(2) << std::setfill('0');
          log() << "REQ packet: servo_id: " << (int)req_pkt.servo_id << " cmd: " << (int)req_pkt.command << " data("
                << req_pkt.data.size() << "): ";
          for (auto c = req_pkt.data.begin(); c != req_pkt.data.end(); c++)
            log() << (int)*c << " ";
          log() << resetfmt << endlog();
        }
      }
      // reset servo poll variables
      if (poll_round_size < 0)
      { // attempt poll all servos in one round
        poll_index = 0;
        poll_end_index = poll_list.size();
      }
      else
      { // poll only poll_round_size servos in current round and preserve poll_index
        if (poll_index >= poll_list.size())
          poll_index = 0; // end of poll list
        poll_end_index = poll_index + poll_round_size;
        if (poll_end_index > poll_list.size())
          poll_end_index = poll_list.size();
      }
      clearPortBuffers();

      if (!waitSendPacketDL.ready())
      {
        waitSendPacketDL();
      }
#ifdef SCHED_STATISTICS
      statistics.rt_jog_send_duration = time_service->secondsSince(statistics_sync_timestamp);
#endif /* SCHED_STATISTICS */

      sched_state = SEND_JOG_WAIT;
      break;
    }
    // Forward all incoming messages to CM interface while waiting sync or before start polling.
    while (!ack_buffer.empty())
    {
      HerkulexPacket* cm_ack_pkt = ack_buffer.PopWithoutRelease();
      if (receivePacketCM.ready())
      {
        receivePacketCM(*cm_ack_pkt);
      }
      ack_buffer.Release(cm_ack_pkt);
    }
    break;

  case SEND_JOG_WAIT:
    // wait for round timer expires
    if (timer.isArmed(ROUND_TIMER))
    {
      break;
    }

    sched_state = SEND_READ_REQ;
    timer.arm(ROUND_TIMER, period_RT_read);
#ifdef SCHED_STATISTICS
    statistics.rt_read_start_time = time_service->secondsSince(statistics_sync_timestamp);
#endif /* SCHED_STATISTICS */

    // check sendPacketDL operation result: possible deadlock detection
    {
      SendStatus result = sendPacketDL_handle.collectIfDone();
      if (result != SendSuccess)
        log(WARN) << "sendPacketDL() operation failed (JOG). SendStatus: " << result << endlog();
    }

  case SEND_READ_REQ:

    if (poll_index >= poll_end_index || !timer.isArmed(ROUND_TIMER))
    {
      // RT round is finished
      timer.killTimer(ROUND_TIMER);

      ros::Time timestamp = ros::Time::now();
      joints.header.stamp = timestamp;
      states.header.stamp = timestamp;

      joints_port.write(joints);
      states_port.write(states);

#ifdef SCHED_STATISTICS
      statistics.cm_start_time = time_service->secondsSince(statistics_sync_timestamp);
      statistics_port.write(statistics);
#endif /* SCHED_STATISTICS */

      // start configuration and monitoring round
      sched_state = CM_ROUND;
      timer.arm(ROUND_TIMER, period_CM);

      log(DEBUG) << "Start CM round." << endlog();

      this->trigger();
      break;
    }
    // form request to servo
    if (!detailed_state)
    {
      success = reqPosVel(req_pkt, poll_list[poll_index]);
    }
    else
    {
      success = reqState(req_pkt, poll_list[poll_index]);
    }
    if (!success)
    {
      // skip servo
      poll_index++;
      this->trigger();
      break;
    }

    timer.arm(REQUEST_TIMEOUT_TIMER, this->timeout);
    // use send mode due to lock bug
    sendPacketDL_handle = sendPacketDL.send(req_pkt);

    if (log(DEBUG))
    {
      log() << std::dec << std::setw(2) << std::setfill('0');
      log() << "REQ packet: servo_id: " << (int)req_pkt.servo_id << " cmd: " << (int)req_pkt.command << " data("
            << req_pkt.data.size() << "): ";
      for (auto c = req_pkt.data.begin(); c != req_pkt.data.end(); c++)
        log() << (int)*c << " ";
      log() << resetfmt << endlog();
    }

    sched_state = RECEIVE_READ_ACK;
    break;

  case RECEIVE_READ_ACK:

    if (!timer.isArmed(REQUEST_TIMEOUT_TIMER))
    {
      poll_index++;
      sched_state = SEND_READ_REQ;
#ifdef SCHED_STATISTICS
      statistics.rt_read_n_errors++;
#endif /* SCHED_STATISTICS */
      log(DEBUG) << "ACK timeout" << endlog();

      // check sendPacketDL result: possible deadlock detection
      {
        SendStatus result = sendPacketDL_handle.collectIfDone();
        if (result != SendSuccess)
          log(WARN) << "sendPacketDL() operation failed (ACK timeout). SendStatus: " << result
                    << " poll_index: " << poll_index - 1 << endlog();
      }

      this->trigger();
      break;
    }

    while (!ack_buffer.empty())
    {
      HerkulexPacket* ack_pkt = ack_buffer.PopWithoutRelease();
      double pos, vel;
      servo::Status status;
      if (!detailed_state)
      {
        success = ackPosVel(*ack_pkt, poll_list[poll_index], pos, vel, status);
      }
      else
      {
        success = ackState(*ack_pkt, poll_list[poll_index], states, status);
        pos = states.pos.back();
        vel = states.vel.back();
      }
      if (success && log(DEBUG))
      {
        log() << std::dec << std::setw(2) << std::setfill('0');
        log() << "ACK packet: servo_id: " << (int)ack_pkt->servo_id << " cmd: " << (int)ack_pkt->command << " data("
              << ack_pkt->data.size() << "): ";
        for (auto c = ack_pkt->data.begin(); c != ack_pkt->data.end(); c++)
          log() << (int)*c << " ";
        log() << resetfmt << std::endl << "pos = " << pos << " vel = " << vel << endlog();
      }
      ack_buffer.Release(ack_pkt);

      if (success)
      {
        // save request results
        joints.name.push_back(poll_list[poll_index]);
        joints.position.push_back(pos);
        joints.velocity.push_back(vel);
        if (!detailed_state)
        {
          states.name.push_back(poll_list[poll_index]);
          states.pos.push_back(pos);
          states.vel.push_back(vel);
          states.error.push_back(status.error);
          states.detail.push_back(status.detail);
        }
#ifdef SCHED_STATISTICS
        statistics.rt_read_req_durationN = timeout - timer.timeRemaining(REQUEST_TIMEOUT_TIMER);
        if (statistics.rt_read_n_successes + statistics.rt_read_n_errors == 0)
        {
          statistics.rt_read_req_duration1 = statistics.rt_read_req_durationN;
        }
        statistics.rt_read_n_successes++;
#endif /* SCHED_STATISTICS */

        // check sendPacketDL result: possible deadlock detection
        {
          SendStatus result = sendPacketDL_handle.collectIfDone();
          if (result != SendSuccess)
            log(WARN) << "sendPacketDL() operation failed (ACK received). SendStatus: " << result << endlog();
        }

        timer.killTimer(REQUEST_TIMEOUT_TIMER);
        poll_index++;
        sched_state = SEND_READ_REQ;
        this->trigger();
        break;
      }
#ifdef SCHED_STATISTICS
      else
      {
        statistics.last_erroneous_status = status;
      }
#endif /* SCHED_STATISTICS */
    }
    break;

  case CM_ROUND:
    if (!timer.isArmed(ROUND_TIMER))
    {
      SchedTimer::TimerId timer_id;
      if (sync_port.read(timer_id) == NewData)
      {
        // we get sync msg before timer expires
        log(ERROR) << "sync message is received before scheduler rounds have been finished." << endlog();
        // now wait for next sync
        // TODO: display statistics
      }
      sched_state = SEND_JOG;
      break;
    }

    // send and receive packets completely asyncronically
    while (!ack_buffer.empty())
    {
      HerkulexPacket* cm_ack_pkt = ack_buffer.PopWithoutRelease();
      if (receivePacketCM.ready())
      {
        receivePacketCM(*cm_ack_pkt);
      }
      ack_buffer.Release(cm_ack_pkt);
    }
    if (!cm_req_buffer.empty())
    {
      HerkulexPacket* cm_req_pkt = cm_req_buffer.PopWithoutRelease();
      sendPacketDL(*cm_req_pkt);
      cm_req_buffer.Release(cm_req_pkt);
      if (!cm_req_buffer.empty())
        this->trigger();
    }
    break;

  } // case (sched_state)
}

void HerkulexSched::receivePacketDL(const HerkulexPacket& pkt)
{
  if (this->isRunning())
  {
    // buffer message to updateHook processing
    ack_buffer.Push(pkt);
    this->trigger();
  }
  else
  {
    // forward message to CM subsystem
    if (receivePacketCM.ready())
    {
      log(DEBUG) << "Forward packet to CM subsytem." << endlog();

      receivePacketCM(pkt);
    }
  }
}

void HerkulexSched::sendPacketCM(const HerkulexPacket& pkt)
{
  if (this->isRunning())
  {
    // buffer message to updateHook processing
    cm_req_buffer.Push(pkt);
    this->trigger();
  }
  else
  {
    // forward message to data link layer
    if (sendPacketDL.ready())
    {
      log(DEBUG) << "Forward packet to DL layer" << endlog();

      sendPacketDL(pkt);
    }
    else
    {
      log(ERROR) << "Data link layer (sendPacketDL operation) is not ready." << endlog();
    }
  }
}

void HerkulexSched::stopHook() { log(INFO) << "HerkulexSched is stopped!" << endlog(); }

void HerkulexSched::cleanupHook() {}

} // namespace herkulex

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
