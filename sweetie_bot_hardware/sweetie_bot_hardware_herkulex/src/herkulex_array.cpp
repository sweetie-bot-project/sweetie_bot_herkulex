#include "herkulex_driver.hpp"

extern "C" {
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
}

#include <vector>

#include <rtt/Component.hpp>
#include <rtt/extras/FileDescriptorActivity.hpp>
#include <rtt/Logger.hpp>


using namespace RTT;

const unsigned int HerkulexDriver::BUFFER_SIZE = 223;
const unsigned int HerkulexDriver::HEADER_SIZE = 7;

HerkulexDriver::HerkulexDriver(std::string const& name) : 
	TaskContext(name),
	cmdAck("cmdAck"),
	port_fd(-1)
{
	this->addOperation("cmdReq", &HerkulexDriver::cmdReq, this, OwnThread) 
		.doc("Send packet to servos.") 
		.arg("pkt", "HerkulexPacket to send.");
	this->requires()->addOperationCaller(cmdAck);

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

	port_fd = open(this->port_name_prop.c_str(), O_RDWR | O_NOCTTY | O_SYNC );
	if (port_fd == -1) {
		log(Error) << "open() serial port \"" << this->port_name_prop << "\" failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// configure serial port
	if (tcgetattr (this->port_fd, &tty) != 0) {
		log(Error) << "tcgetattr() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	/*
	// 8-bits, 1 STOP bit, enable receiver, ignore modem lines
	tty.c_cflag = CS8 | CREAD | CLOCAL; 
	// no signaling chars, no echo, no canonical processing
	tty.c_lflag = 0;
	// no special input processing
	tty.c_iflag = 0;
	// no special output processing
	tty.c_oflag = 0;*/
	// 8-bits, 1 STOP bit, enable receiver, ignore modem lines
	cfmakeraw(&tty);
	tty.c_cflag = CREAD | CLOCAL; 
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
			log(Error) << "Incorrect baudrate property value: " << baudrate_prop << endlog(); 
			return false;
	}
	if (ret) {
		log(Error) << "cfsetspeed() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// special properties
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout, so read will not block forever
	// configure port
	if (tcsetattr (this->port_fd, TCSANOW, &tty) != 0) {
		log(Error) << "tcsetattr() failed: " << strerror(errno) << endlog(); 
		return false;
	}
	// Setup FileDescriptorActivity
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(Error) << "Incompatible activity type."  << endlog(); 
	}
	activity->watch(port_fd);
	// reserve memory
	recv_pkt.data.reserve(BUFFER_SIZE - HEADER_SIZE);
	return true;
}

bool HerkulexDriver::startHook()
{
	// flush input buffer
	tcflush(port_fd, TCIFLUSH);
	// wait for header
	recv_state = HEADER1;
}

void HerkulexDriver::updateHook()
{
	extras::FileDescriptorActivity * activity = dynamic_cast<extras::FileDescriptorActivity *>(this->getActivity());
	if (! activity) {
		log(Error) << "Incompatible activity type."  << endlog(); 
		this->exception();
	}
	if (activity->hasError()) {
		log(Error) << "FileDescriptorActivity error."  << endlog(); 
		this->exception();
	}

	if (activity->isUpdated(port_fd)) {
		// read new data on port
		char buffer[BUFFER_SIZE];
		ssize_t buffer_index, buffer_size;

		ssize_t retval = TEMP_FAILURE_RETRY(read(port_fd, buffer, sizeof(buffer)));
		if (retval == 0) return;
		else if (retval == -1) {
			log(Error) << "Read serial port failed:" << strerror(errno) << endlog(); 
			this->exception();
		}
		buffer_size = retval;
		buffer_index = 0;

		// parse new data in buffer
		for(; buffer_index < buffer_size; buffer_index++) {
			char c = buffer[buffer_index];

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
						case herkulex_msgs::HerkulexPacket::ACK_EEP_READ:
						case herkulex_msgs::HerkulexPacket::ACK_RAM_READ:
						case herkulex_msgs::HerkulexPacket::ACK_STAT:
						case herkulex_msgs::HerkulexPacket::ACK_ROLLBACK:
						case herkulex_msgs::HerkulexPacket::ACK_REBOOT:
							recv_pkt.command = c;
							recv_state = CHECKSUM1;
							break;
						default:
							recv_pkt.command = c;
							log(Warning) << "ACK packet type is unknown, servo = " << std::hex << recv_pkt.servo_id << " cmd = " << recv_pkt.command << std::dec << endlog();
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
						log(Warning) << "ACK packet, checksum2 error, servo = " << std::hex << recv_pkt.servo_id << " cmd = " << recv_pkt.command << std::dec << endlog();
						recv_state = HEADER1;
						break;
					}
					recv_state = DATA;
					break;

				case DATA:
					{
						ssize_t bytes_to_read = recv_pkt_size - HEADER_SIZE - recv_pkt.data.size();
						bytes_to_read = std::min(bytes_to_read, buffer_size - buffer_index);

						recv_pkt.data.insert(recv_pkt.data.end(), buffer + buffer_index, buffer + buffer_index + bytes_to_read);

						buffer_index += bytes_to_read - 1;
					}
					// check if full packet is receved
					if (recv_pkt.data.size() == recv_pkt_size - HEADER_SIZE) {
						// checksum check
						unsigned char checksum = recv_pkt_size ^ recv_pkt.servo_id ^ recv_pkt.command;
						for(int i = 0; i < recv_pkt.data.size(); i++) checksum ^= recv_pkt.data[i];
						checksum &= 0xFE;

						if (checksum == recv_pkt_checksum1) {
							// send packet
							log(RealTime) << "ACK packet is successfully received: servo = " << std::hex << recv_pkt.servo_id << " cmd = " << recv_pkt.command << std::dec << endlog();
							this->cmdAck(recv_pkt);
						}
						else {
							log(Warning) << "ACK packet checksum1 error, servo = " << std::hex << recv_pkt.servo_id << " cmd = " << recv_pkt.command << std::dec << endlog();
						}

						recv_state = HEADER1;
					}
					break;
			}
		}
	} // if (activity->isUpdated(port_fd))
}

bool HerkulexDriver::cmdReq(const herkulex_msgs::HerkulexPacket& pkt) 
{
	char buffer[BUFFER_SIZE];
	size_t pkt_size = HEADER_SIZE + pkt.data.size();

	if (pkt_size > BUFFER_SIZE) {
		log(Error) << "REQ packet is too large." << endlog(); 
		return false;
	}

	// prepare send buffer
	buffer[0] = 0xFF;
	buffer[1] = 0xFF;
	buffer[2] = pkt_size;
	buffer[3] = pkt.servo_id;
	buffer[4] = pkt.command;

	unsigned char xor_accum = buffer[2] ^ buffer[3] ^ buffer[4];
	for(int i = 0; i < pkt.data.size(); i++) {
		buffer[i + HEADER_SIZE] = pkt.data[i];
		xor_accum ^= pkt.data[i];
	}

	buffer[5] = xor_accum & 0xFE;
	buffer[6] = ~buffer[5] & 0xFE;

	size_t bytes_written = 0;
	do {
		ssize_t retval = TEMP_FAILURE_RETRY(write(port_fd, buffer + bytes_written, pkt_size - bytes_written));
		if (retval == -1) {
			log(Error) << "Write to serial port failed: " << strerror(errno) << endlog(); 
			this->exception();
			return false;
		}
		bytes_written += retval;
	}
	while (bytes_written < pkt_size);

	log(RealTime) << "REQ packet is successfully sended: servo = " << std::hex << pkt.servo_id << " cmd = " << pkt.command << std::dec << endlog();
	return true;
}

void HerkulexDriver::stopHook() 
{
}

void HerkulexDriver::cleanupHook() {
	if (TEMP_FAILURE_RETRY(close(port_fd))) {
		log(Error) << "close() serial port failed: " << strerror(errno) << endlog(); 
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
ORO_CREATE_COMPONENT(HerkulexDriver)
