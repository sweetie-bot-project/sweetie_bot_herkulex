#include "herkulex_driver.hpp"

extern "C" {
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
}

#include <vector>

#include <rtt/Component.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <rtt/os/TimeService.hpp>


using namespace sweetie_bot;
using namespace RTT;

namespace herkulex
{

//Convinence macro fo logging.
std::ostream& resetfmt(std::ostream& s) {
	s.copyfmt(std::ios(NULL)); 
	return s;
}

HerkulexDriver::HerkulexDriver(std::string const& name) : 
	TaskContext(name, PreOperational),
	receivePacketDL("receivePacket"),
	port_fd(-1),
	log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("HerkulexDriver");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
		this->fatal();
		return;
	}

	this->addOperation("sendPacket", &HerkulexDriver::sendPacketDL, this, OwnThread) 
		.doc("Send packet to servos.") 
		.arg("pkt", "HerkulexPacket to send.");

	this->addOperation("waitSendPacket", &HerkulexDriver::waitSendPacketDL, this, OwnThread) 
		.doc("Wait until last send packet is actually writen to serial port.");

	this->requires()->addOperationCaller(receivePacketDL);

	this->addProperty("port_name", port_name_prop) 
		.doc("Serial port device.");
	this->addProperty("baudrate", baudrate_prop)
		.doc("Serial port baudrate.")
		.set(115200);

	this->setActivity(new extras::FileDescriptorActivity(60, 0, "HerkulexPortActivity"));
}

bool HerkulexDriver::configureHook()
{
	struct termios tty;

	port_fd = open(this->port_name_prop.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK );
	if (port_fd == -1) {
		log(ERROR) << "open() serial port \"" << this->port_name_prop << "\" failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// configure serial port
	if (tcgetattr (this->port_fd, &tty) != 0) {
		log(ERROR) << "tcgetattr() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	
	// 8-bits, 1 STOP bit, enable receiver, ignore modem lines
	//tty.c_cflag = CS8 | CREAD | CSTOPB | CLOCAL; 
	tty.c_cflag = CS8 | CREAD | CLOCAL; 
	// no signaling chars, no echo, no canonical processing
	tty.c_lflag = 0;
	// no special input processing
	tty.c_iflag = 0;
	// no special output processing
	tty.c_oflag = 0;
	// set speed
	int ret;
	switch (this->baudrate_prop) {
		case 9600:
			ret = cfsetspeed (&tty, B9600);
			break;
		case 19200:
			ret = cfsetspeed (&tty, B19200);
			break;
		case 38400:
			ret = cfsetspeed (&tty, B38400);
			break;
		case 57600:
			ret = cfsetspeed (&tty, B57600);
			break;
		case 115200:
			ret = cfsetspeed (&tty, B115200);
			break;
		case 230400:
			ret = cfsetspeed (&tty, B230400);
			break;
		default:
			log(ERROR) << "Incorrect baudrate property value: " << baudrate_prop << endlog(); 
			return false;
	}
	if (ret) {
		log(ERROR) << "cfsetspeed() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// special properties
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout, so read will not block forever
	// configure port
	if (tcsetattr (this->port_fd, TCSANOW, &tty) != 0) {
		log(ERROR) << "tcsetattr() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// set low_latency flag
	struct serial_struct serial;
	if (ioctl(this->port_fd, TIOCGSERIAL, &serial) == -1) {
		log(WARN) << "Unable to get serial_struct. ioctl() failed: " << strerror(errno) << endlog();
	}
	else {
		serial.flags |= ASYNC_LOW_LATENCY;
		if (ioctl(this->port_fd, TIOCSSERIAL, &serial) == -1) {
			log(WARN) << "Unable to set low_latency flag. ioctl() failed: " << strerror(errno) << endlog();
		}
	}
	// Setup FileDescriptorActivity
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(ERROR) << "Incompatible activity type."  << endlog(); 
		return false;
	}
	activity->watch(port_fd);
	
	// reserve memory
	recv_pkt.data.reserve(HerkulexPacket::DATA_SIZE);
	log(INFO) << "HerkulexDriver is configured!" << endlog(); 
	return true;
}

bool HerkulexDriver::startHook()
{
	// flush input buffer
	int retval = tcflush(port_fd, TCIFLUSH);
	if (retval == -1) {
		log(ERROR) << "tcflush() failed:" << strerror(errno) << endlog(); 
		return false;
	}
	// wait for header
	recv_state = HEADER1;
	log(INFO) << "HerkulexDriver is started!" << endlog(); 
	return true;
}

void HerkulexDriver::updateHook()
{
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(ERROR) << "Incompatible activity type."  << endlog(); 
		this->exception();
		return;
	}
	if (activity->hasError()) {
		log(ERROR) << "FileDescriptorActivity error."  << endlog(); 
		this->exception();
		return;
	}

	if (activity->isUpdated(port_fd)) {
		// read new data on port
		unsigned char buffer[HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE];
		ssize_t buffer_index, buffer_size;

		ssize_t retval = TEMP_FAILURE_RETRY(read(port_fd, buffer, sizeof(buffer)));
		if (retval == 0) return;
		else if (retval == -1) {
			log(ERROR) << "Read serial port failed:" << strerror(errno) << endlog(); 
			this->exception();
		}
		buffer_size = retval;
		buffer_index = 0;

		if (log(DEBUG)) {
			log() << "READ on serial port (" << buffer_size << " bytes):" << std::hex << std::setw(2) << std::setfill('0');
			for (int i = 0; i < buffer_size; i++) log(DEBUG) << (unsigned int) buffer[i] << " ";
			log() << resetfmt << std::endl << "STATE = " << recv_state << endlog();
		}

		// parse new data in buffer
		for(; buffer_index < buffer_size; buffer_index++) {
			unsigned char c = buffer[buffer_index];

			/*if (log().getLogLevel() >= Logger::DEBUG) {
				Logger::In in("HerkulexDriver");
				log(DEBUG) << "c = " << std::hex << (unsigned int) c << std::dec << " state = " << recv_state << endlog();
			}*/

			switch (recv_state) {
				case HEADER1:
					//TODO use memchr
					if (c == 0xFF) recv_state = HEADER2;
					break;

				case HEADER2:
					if (c != 0xFF) recv_state = HEADER1;	
					recv_pkt.data.clear();
					recv_state = PACKET_SIZE;
					break;

				case PACKET_SIZE:
					recv_pkt_size = c;
					recv_state = SERVO_ID;
					break;

				case SERVO_ID:
					recv_pkt.servo_id = c;
					recv_state = CMD;
					break;

				case CMD:
					switch (c) {
						//TODO ACK mask
						case HerkulexPacket::ACK_EEP_READ:
						case HerkulexPacket::ACK_RAM_READ:
						case HerkulexPacket::ACK_EEP_WRITE:
						case HerkulexPacket::ACK_RAM_WRITE:
						case HerkulexPacket::ACK_S_JOG:
						case HerkulexPacket::ACK_I_JOG:
						case HerkulexPacket::ACK_STAT:
						case HerkulexPacket::ACK_ROLLBACK:
						case HerkulexPacket::ACK_REBOOT:
							recv_pkt.command = c;
							recv_state = CHECKSUM1;
							break;
						default:
							{
								log(WARN) << "ACK packet type is unknown, servo = " << std::hex << (unsigned int) recv_pkt.servo_id << " cmd = " << (unsigned int) recv_pkt.command << std::dec << endlog();
							}
							recv_state = HEADER1;
							break;
					}
					break;

				case CHECKSUM1:
					recv_pkt_checksum1 = c;
					recv_state = CHECKSUM2;
					break;

				case CHECKSUM2:
					if ( c != (~recv_pkt_checksum1 & 0xFE) ) {
						{
							log(WARN) << "ACK packet, checksum2 error, servo = " << std::hex << (unsigned int) recv_pkt.servo_id << " cmd = " << (unsigned int) recv_pkt.command << std::dec << endlog();
						}
						recv_state = HEADER1;
						break;
					}
					recv_state = DATA;
					break;

				case DATA:
					{
						ssize_t bytes_to_read = recv_pkt_size - HerkulexPacket::HEADER_SIZE - recv_pkt.data.size();
						bytes_to_read = std::min(bytes_to_read, buffer_size - buffer_index);

						recv_pkt.data.insert(recv_pkt.data.end(), buffer + buffer_index, buffer + buffer_index + bytes_to_read);

						buffer_index += bytes_to_read - 1;
					}
					// check if full packet is receved
					if (recv_pkt.data.size() == recv_pkt_size - HerkulexPacket::HEADER_SIZE) {
						// checksum check
						unsigned char checksum = recv_pkt_size ^ recv_pkt.servo_id ^ recv_pkt.command;
						for(int i = 0; i < recv_pkt.data.size(); i++) checksum ^= recv_pkt.data[i];
						checksum &= 0xFE;

						if (checksum == recv_pkt_checksum1) {
							// send packet
							if (receivePacketDL.ready()) {
								receivePacketDL(recv_pkt);
							}
						}
						else {
							log(WARN) << "ACK packet checksum1 error, servo = " << std::hex << (unsigned int) recv_pkt.servo_id << " cmd = " << (unsigned int) recv_pkt.command << std::dec << endlog();
						}
						recv_state = HEADER1;
					}
					break;
			}
		}
	} // if (activity->isUpdated(port_fd))
}

void HerkulexDriver::sendPacketDL(const HerkulexPacket& pkt) 
{
	unsigned char buffer[HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE];
	size_t pkt_size = HerkulexPacket::HEADER_SIZE + pkt.data.size();

	if (!isConfigured()) return;

	if (pkt_size > HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE) {
		log(ERROR) << "REQ packet is too large." << endlog(); 
		return;
	}

	// prepare send buffer
	// header
	buffer[0] = 0xFF;
	buffer[1] = 0xFF;
	buffer[2] = pkt_size;
	buffer[3] = pkt.servo_id;
	buffer[4] = pkt.command;

	// packet data and checksum calculation
	unsigned char xor_accum = buffer[2] ^ buffer[3] ^ buffer[4];
	for(int i = 0; i < pkt.data.size(); i++) {
		buffer[i + HerkulexPacket::HEADER_SIZE] = pkt.data[i];
		xor_accum ^= pkt.data[i];
	}

	buffer[5] = xor_accum & 0xFE;  // checksum1
	buffer[6] = ~buffer[5] & 0xFE; // checksum2

	size_t bytes_written = 0;
	do {
		ssize_t retval = TEMP_FAILURE_RETRY(write(port_fd, buffer + bytes_written, pkt_size - bytes_written));
		if (retval == -1) {
			log(ERROR) << "Write to serial port failed: " << strerror(errno) << endlog(); 
			this->exception();
			return;
		}
		bytes_written += retval;
	}
	while (bytes_written < pkt_size);

	//TODO make this optional
	// reset receiver in case future reply
	recv_state = HEADER1;

	if (log(DEBUG)) {
		log() << "WRITE on serial port (" << pkt_size << " bytes):" << std::hex << std::setw(2) << std::setfill('0');
		for (int i = 0; i < pkt_size; i++) log(DEBUG) << (unsigned int) buffer[i] << " ";
		log() << resetfmt << endlog();
	}

}

void HerkulexDriver::waitSendPacketDL() 
{
	// Wait until all data is written to port.
	if (TEMP_FAILURE_RETRY(tcdrain(port_fd)) == -1) {
		log(ERROR) << "tcdrain() failed: " << strerror(errno) << endlog(); 
		this->exception();
		return;
	}
}

void HerkulexDriver::stopHook() 
{
	log(INFO) << "HerkulexDriver is stopped!" << endlog(); 
}

void HerkulexDriver::cleanupHook() {
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(ERROR) << "Incompatible activity type."  << endlog(); 
	}
	activity->unwatch(port_fd);
	if (TEMP_FAILURE_RETRY(close(port_fd))) {
		log(ERROR) << "close() serial port failed: " << strerror(errno) << endlog(); 
	}
	log(INFO) << "HerkulexDriver is cleaned up!" << endlog(); 
}

}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(HerkulexDriver)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(herkulex::HerkulexDriver)
