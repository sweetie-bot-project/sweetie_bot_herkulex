#include <stdexcept>
#include "herkulex_servo.hpp"

namespace herkulex {

namespace servo {

RegisterMapper::RegisterMapper(const std::vector<Register>& _regs) :
	registers(_regs)
{
	for(std::vector<Register>::const_iterator reg = registers.begin(); reg != registers.end(); reg++) {
		if (reg->reg_num != reg - registers.begin()) throw std::invalid_argument("RegisterMapper: register array is not uniform at register \'" + reg->name + "\'.");
		name_map[reg->name] = &(*reg);
		//TODO check if memory is uniform
		if (reg->eep_addr != -1) eep_map[reg->eep_addr] = &(*reg);
		if (reg->ram_addr != -1) ram_map[reg->ram_addr] = &(*reg);
	};
}

HerkulexServo::HerkulexServo(const std::string& _name, const RegisterMapper& _mapper, unsigned int _hw_id, bool _reverse, int _offset) :
	register_mapper(_mapper),
	name(_name),
	hw_id(_hw_id),
	reverse(_reverse),
	offset(_offset)
{
}

void HerkulexServo::reqRead_ram(HerkulexPacket& req, const std::string& reg) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	const Register& r = register_mapper.getByName(reg);
	if (r.ram_addr == -1) throw std::out_of_range("HerkulexServo::ackRead_ram: register '" + r.name + "' does not present in RAM region of servo memory.");
	req.data[0] = r.ram_addr;
	req.data[1] = r.bytes;
}


void HerkulexServo::reqRead_eep(HerkulexPacket& req, const std::string& reg) const
{
	req.command = HerkulexPacket::REQ_EEP_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	const Register& r = register_mapper.getByName(reg);
	if (r.eep_addr == -1) throw std::out_of_range("HerkulexServo::eepRead_eep: register '" + r.name + "' does not present in EEP region of servo memory.");
	req.data[0] = r.eep_addr;
	req.data[1] = r.bytes;
}
					
void HerkulexServo::reqWrite_impl(HerkulexPacket& req, unsigned int reg_num, unsigned int n, const unsigned int * data) const 
{
	req.data.resize(2);
	unsigned int size = 0;
	for(unsigned int i = 0; i < n; i++) {
		switch ( register_mapper.getByNum(reg_num + i).bytes ) {
			case 1:
				req.data.push_back(data[i]);
				size += 1;
				break;
			case 2:
				req.data.push_back(data[i] & 0xff);
				req.data.push_back((data[i] >> 8) & 0xff);
				size += 2;
				break;
			default:
				throw std::domain_error("HerkulexServo::ackWrite: incompatible servo register size.");
				break;
		}
	}
	req.data[0] = 0;
	req.data[1] = size;
}

void HerkulexServo::reqWrite_ram(HerkulexPacket& req, const std::string& reg, unsigned int val) const
{
	req.servo_id = hw_id;
	req.command = HerkulexPacket::REQ_RAM_WRITE;
	const Register& r = register_mapper.getByName(reg);
	if (r.ram_addr == -1) throw std::out_of_range("HerkulexServo::ackWrite_ram: register '" + r.name + "' does not present in RAM region of servo memory.");
	if (! r.rw) throw std::out_of_range("HerkulexServo::ackWrite_ram: register '" + r.name + "' is not writable.");
	reqWrite_impl(req, r.reg_num, 1, &val);
	req.data[0] = r.ram_addr;
}

void HerkulexServo::reqWrite_eep(HerkulexPacket& req, const std::string& reg, unsigned int val) const 
{
	req.servo_id = hw_id;
	req.command = HerkulexPacket::REQ_EEP_WRITE;
	const Register& r = register_mapper.getByName(reg);
	if (r.eep_addr == -1) throw std::out_of_range("HerkulexServo::ackWrite_eep: register '" + r.name + "' does not present in RAM region of servo memory.");
	if (! r.rw) throw std::out_of_range("HerkulexServo::ackWrite_eep: register '" + r.name + "' is not writable.");
	reqWrite_impl(req, r.reg_num, 1, &val);
	req.data[0] = r.eep_addr;
}

void HerkulexServo::reqStat(HerkulexPacket& req) const 
{
	req.command = HerkulexPacket::REQ_STAT;
	req.servo_id = hw_id;
	req.data.resize(0);
}

void HerkulexServo::reqRollback(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_ROLLBACK;
	req.servo_id = hw_id;
	req.data.resize(0);
}

void HerkulexServo::reqReset(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_REBOOT;
	req.servo_id = hw_id;
	req.data.resize(0);
}

void HerkulexServo::reqWriteClearStatus(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_WRITE;
	req.servo_id = hw_id;
	req.data.resize(4);
	req.data[0] = register_mapper.getByName("status_error").ram_addr;
	req.data[1] = 2;
	req.data[2] = 0;
	req.data[3] = 0;
}

bool HerkulexServo::ackRead_impl(const HerkulexPacket& ack, unsigned int reg_num, unsigned int * data, Status& status) const 
{
	unsigned int index = 2;
	while(index < ack.data.size() - 2)
	{
		switch (register_mapper.getByNum(reg_num).bytes) {
			case 1:
				*data++ =  ack.data[index];
				index += 1;
				break;
			case 2:
				*data++ = ack.data[index] + (static_cast<unsigned int>(ack.data[index+1]) << 8);
				index += 2;
				break;
			default:
				throw std::domain_error("HerkulexServo::ackRead: incompatible servo register size.");
				break;
		}
	}
	if (index != ack.data.size() - 2) return false;
	status.error = ack.data[index];
	status.detail = ack.data[index+1];
	return true;
}

bool HerkulexServo::ackRead_ram(const HerkulexPacket& ack, const std::string& reg, unsigned int& val, Status& status) const 
{
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() <= 4) return false;
	unsigned int addr = ack.data[0];
	const Register * r = register_mapper.findByRAMaddr(addr);
	if (r == nullptr) return false;
	if (r->name != reg) return false;
	if (ack.data.size() != 4 + r->bytes) return false;
	return ackRead_impl(ack, r->reg_num, &val, status);
}

bool HerkulexServo::ackRead_eep(const HerkulexPacket& ack, const std::string& reg, unsigned int& val, Status& status) const 
{
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_EEP_READ) return false;
	if (ack.data.size() <= 4) return false;
	unsigned int addr = ack.data[0];
	const Register * r = register_mapper.findByEEPaddr(addr);
	if (r == nullptr) return false;
	if (r->name != reg) return false;
	if (ack.data.size() != 4 + r->bytes) return false;
	return ackRead_impl(ack, r->reg_num, &val, status);
}
	
bool HerkulexServo::ackStatReturn_impl(const HerkulexPacket& ack, Status& status) const 
{
	if (ack.servo_id != hw_id) return false;
	if (ack.data.size() != 2) return false;
	status.error = ack.data[0];
	status.detail = ack.data[1];
	return true;
}

void HerkulexServo::reqIJOGheader(HerkulexPacket& req) const
{
	req.servo_id = hw_id;
	req.command = HerkulexPacket::REQ_I_JOG;
	req.data.resize(0);
}

void HerkulexServo::insertIJOGdata(HerkulexPacket& req, JOGMode mode, unsigned int goal, unsigned int playtime) const
{
	req.data.push_back(goal & 0xFF); // LSB goal
	req.data.push_back((goal >> 8) & 0xFF); // LSB goal
	req.data.push_back(mode & JOGMode::JOGMODE_MASK); // mode
	req.data.push_back(hw_id); // ID
	req.data.push_back(playtime); //playtime
}

void HerkulexServo::reqSJOGheader(HerkulexPacket& req, unsigned int playtime) const
{
	req.servo_id = hw_id;
	req.command = HerkulexPacket::REQ_S_JOG;
	req.data.resize(1);
	req.data[0] = playtime;
}

void HerkulexServo::insertSJOGdata(HerkulexPacket& req, JOGMode mode, unsigned int goal) const
{
	req.data.push_back(goal & 0xFF); // LSB goal
	req.data.push_back((goal >> 8) & 0xFF); // LSB goal
	req.data.push_back(mode); // mode
	req.data.push_back(hw_id); // ID
}

} // namespace servo

}// namespace herkulex
