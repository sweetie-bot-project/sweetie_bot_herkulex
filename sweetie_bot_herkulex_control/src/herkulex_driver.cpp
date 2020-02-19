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
	log(logger::categoryFromComponentName(name)),
	send_log(logger::categoryFromComponentName(name))
{
	if (!log.ready()) {
		RTT::Logger::In in("HerkulexDriver");
		RTT::log(RTT::Error) << "Logger is not ready!" << RTT::endlog();
		this->fatal();
		return;
	}

	this->addOperation("sendPacket", &HerkulexDriver::sendPacketDL, this, ClientThread)  // see note about 'ClientThread' in function definition
		.doc("Send packet to servos.") 
		.arg("pkt", "HerkulexPacket to send.");

	this->addOperation("waitSendPacket", &HerkulexDriver::waitSendPacketDL, this, ClientThread)  // see note about 'ClientThread'  in function definition
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
	// clear receive buffers
	recv_buffer_index = 0;
	recv_buffer_header_index = 0;
	recv_buffer_size = 0;
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
		// read new data on port and add it to raw buffer
		ssize_t retval = TEMP_FAILURE_RETRY(read(port_fd, recv_buffer + recv_buffer_size, sizeof(recv_buffer) - recv_buffer_size));
		if (retval == 0) return;
		else if (retval == -1) {
			log(ERROR) << "Read serial port failed:" << strerror(errno) << endlog(); 
			this->exception();
			return;
		}
		recv_buffer_size += retval;

		if (log(DEBUG)) {
			log() << "RECEIVE buffer (" << recv_buffer_size << " bytes): ";
			for (int i = 0; i < recv_buffer_size; i++)
				log() << std::setfill('0') << std::setw(2) << std::right << std::hex << unsigned(recv_buffer[i]) << " ";
			log() << endlog();
		}

		// parse new data in buffer
		while (recv_buffer_index < recv_buffer_size) {
			unsigned char c;
			unsigned char * header_ptr;

			// bool gc(unsigned char& c) {
			//    if (recv_buffer_index >= recv_buffer_size) return false;
			//    c = recv_buffer[recv_buffer_index];
			//    recv_buffer_index++;
			//    return true;
			// }

			if (log(DEBUG)) {
				log() << "RECEIVER state = " << recv_state << " index = " << recv_buffer_index << " header_index = " << recv_buffer_header_index << endlog();
			}

			switch (recv_state) {
				case HEADER1:
					header_ptr = (unsigned char *) memchr(recv_buffer + recv_buffer_index, 0xFF, recv_buffer_size - recv_buffer_index);
					if (header_ptr == nullptr) {
						// header not found: clear receive buffer
						recv_buffer_header_index = recv_buffer_size;
						recv_buffer_index = recv_buffer_size; 
						break;
					}
					// 0xFF found: store header start position
					recv_buffer_header_index = header_ptr - recv_buffer;
					recv_buffer_index = recv_buffer_header_index + 1; // first non-processed byte

					recv_state = HEADER2;

				case HEADER2:
					if (!gc(c)) break;

					if (c != 0xFF) {
						recv_state = HEADER1;
						break;
					}
					recv_pkt.data.clear();
					recv_state = PACKET_SIZE;

				case PACKET_SIZE:
					if (!gc(c)) break;

					if (c > HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE) {
						if (c == 0xFF) {
							// 0xFF 0XFF sequence is still present, but we have to shift header position one byte forward
							recv_buffer_header_index++;
							break; 
						}
						else {
							// incorrect packet size
							log(WARN) << "ACK packet size is incorrect, size = " << std::dec << (unsigned int) c <<  endlog();
							recv_state = PARSE_ERROR;
							break;
						}
					}
					recv_pkt_size = c;
					recv_state = SERVO_ID;

				case SERVO_ID:
					if (!gc(c)) break;

					recv_pkt.servo_id = c;
					recv_state = CMD;

				case CMD:
					if (!gc(c)) break;

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
							log(WARN) << "ACK packet type is unknown, servo = " << std::dec << (unsigned int) recv_pkt.servo_id << " cmd = " << (unsigned int) recv_pkt.command << std::dec << endlog();
							recv_state = PARSE_ERROR;
							break;
					}
					break;

				case CHECKSUM1:
					if (!gc(c)) break;

					recv_pkt_checksum1 = c;
					recv_state = CHECKSUM2;

				case CHECKSUM2:
					if (!gc(c)) break;

					if ( c != (~recv_pkt_checksum1 & 0xFE) ) {
						log(WARN) << "ACK packet, checksum2 error, servo = " << std::dec << (unsigned int) recv_pkt.servo_id << " cmd = " << (unsigned int) recv_pkt.command << std::dec << endlog();
						recv_state = PARSE_ERROR;
						break;
					}
					recv_state = DATA;

				case DATA:
					{
						ssize_t bytes_to_read = recv_pkt_size - HerkulexPacket::HEADER_SIZE - recv_pkt.data.size();
						bytes_to_read = std::min(bytes_to_read, recv_buffer_size - recv_buffer_index);

						recv_pkt.data.insert(recv_pkt.data.end(), recv_buffer + recv_buffer_index, recv_buffer + recv_buffer_index + bytes_to_read);

						recv_buffer_index += bytes_to_read; // first non-processed byte
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
							recv_buffer_header_index = recv_buffer_index; // skip everysing until next byte
							recv_state = HEADER1;
						}
						else {
							log(WARN) << "ACK packet checksum1 error, servo = " << std::dec << (unsigned int) recv_pkt.servo_id << " cmd = " << (unsigned int) recv_pkt.command << std::dec << endlog();
							recv_state = PARSE_ERROR;
						}
					}
					break;

				case PARSE_ERROR:
					// parse error occurs: start buffer analisys from the second byte of header
					recv_buffer_index = recv_buffer_header_index + 1; 
					recv_state = HEADER1;
					break;
			}
		} 

		// shift buffer content to clear everything until header start
		if (recv_buffer_header_index > 0) {
			recv_buffer_size -= recv_buffer_header_index;
			recv_buffer_index -= recv_buffer_header_index;
			memmove(recv_buffer, recv_buffer + recv_buffer_header_index, recv_buffer_size);
		}

		if (recv_buffer_size == sizeof(recv_buffer)) {
			log(FATAL) << "Receiver buffer depleded. Packet parser internal logic error." << endlog();
			this->exception();
			return;
		}
	} // if (activity->isUpdated(port_fd))
}

void HerkulexDriver::sendPacketDL(const HerkulexPacket& pkt) 
{
	// This operation is called in client thread with corresponding priority.
	//
	// This is walkaround for deadlock bug, when caller is blocked during call to sendPacketDL() operation 
	// (It seems it does not receive notification after operation is executed). This bug is a quite rare occurence, 
	// but eventually it blocks caller if it works long enough.
	//
	// send_mutex prevent races realted with write syscall and logger.
	// This function uses separate logger instance (send_log) to avoid collisions with updateHook().

	if (!isConfigured()) return;

	// lock mutex to prevent simultenous write and logging attempt
	RTT::os::MutexLock lock(send_mutex);

	unsigned char buffer[HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE];
	size_t pkt_size = HerkulexPacket::HEADER_SIZE + pkt.data.size();

	if (pkt_size > HerkulexPacket::HEADER_SIZE + HerkulexPacket::DATA_SIZE) {
		send_log(ERROR) << "REQ packet is too large." << endlog();
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
			RTT::os::MutexLock lock(send_mutex); // prevent simultenous logging attempts
			send_log(ERROR) << "Write to serial port failed: " << strerror(errno) << endlog(); 
			this->exception();
			return;
		}
		bytes_written += retval;
	}
	while (bytes_written < pkt_size);

	if (send_log(DEBUG)) {
		send_log() << "WRITE on serial port (" << pkt_size << " bytes): ";
		for (int i = 0; i < pkt_size; i++)
			send_log() << std::setfill('0') << std::setw(2) << std::right << std::hex << unsigned(buffer[i]) << " ";
		send_log() << resetfmt << endlog();
	}
}

void HerkulexDriver::waitSendPacketDL() 
{
	// Wait until all data is written to port.
	// See comment in for sendPacketDL().
	if (TEMP_FAILURE_RETRY(tcdrain(port_fd)) == -1) {
		send_log(ERROR) << "tcdrain() failed: " << strerror(errno) << endlog();
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
