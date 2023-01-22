#include "sweetie_bot_servo_sbs01.hpp"

#include <cstdint>

namespace herkulex {

namespace servo {


const unsigned int  SweetiBotServoSBS01::POS_RAW_MAX = 4095;

const double SweetiBotServoSBS01::POS_CONV_COEFF_RAW2RAD = M_PI/180.0*320.0/4096.0;

const double SweetiBotServoSBS01::VEL_CONV_COEFF_RAW2RADS = POS_CONV_COEFF_RAW2RAD * 16000.0 / (1 << 14);

const double SweetiBotServoSBS01::VOLTAGE_CONV_COEFF_RAW2VOLT = 0.0088623;

const double SweetiBotServoSBS01::EFFORT_CONV_COEFF_RAW2HM = 0.00148 * (4.5 / 3.9);

const double SweetiBotServoSBS01::TIME_CONV_COEFF_RAW2SEC = 0.001;

const std::vector<Register> SweetiBotServoSBS01::registers =
{
//num    name                        eep  ram bytes   rw      description
{   0, "model_no_1",                       0,  -1,   1, false, "Hardware model (major)." },
{   1, "model_no_2",                       1,  -1,   1, false, "Hardware model (minor)." },
{   2, "version1",                         2,  -1,   1, false, "Firmware version (major)." },
{   3, "version2",                         3,  -1,   1, false, "Firmware version (minor)." },
{   4, "baudrate",                         4,  -1,   1,  true, "Baudrate: 0x1 (57600), 0x2 (115200), 0x3 (1000000), 0x4 (2000000)." },
{   5, "id",                               5,  -1,   1,  true, "Servo ID." },
{   6, "ack_policy",                       6,   0,   1,  true, "ACK packet send policy: 0x01 -- send READ ACK, 0x2 -- WRITE ACK, 0x04 ERROR ACK, 0x10 -- RT_EXCHANGE ACK." },
{   7, "torque_policy",                    7,   1,   1,  true, "If (r{torque_policy} & r{status_error} & 0x7F) set r{torque_mode} to 0x00 or 0x40 (depending on (r{torque_policy} & 0x80)" },
{   8, "max_temperature",                  8,   2,   1,  true, "Overhead detection threshold (16 ADC units)." },
{   9, "min_voltage",                      9,   3,   1,  true, "Undervoltage detection threshold (16 ADC units)." },
{  10, "max_voltage",                     10,   4,   1,  true, "Overvoltage detection threshold (16 ADC units)." },
{  11, "avg_current_alpha",               11,   5,   1,  true, "Average current filter coefficient (u0.16) on T2 period." },
{  12, "max_avg_current",                 12,   6,   2,  true, "Average overcurrent detection threshold (ADC units)." },
{  13, "max_current",                     14,   8,   2,  true, "Peak overcurrent detection threshold (ADC units)." },
{  14, "current_limit",                   16,  10,   2,  true, "Maximal allowed reference current (saturation limit) (ADC units)." },
{  15, "pwm_limit",                       18,  12,   2,  true, "Maximal allowed PWM." },
{  16, "min_position",                    20,  14,   2,  true, "Movement limit absolute position. Check is performed after applying position offset." },
{  17, "max_position",                    22,  16,   2,  true, "Movement limit absolute position. Check is performed after applying position offset." },
{  18, "position_offset",                 24,  18,   2,  true, "Position offset, ADC units (u16)" },
{  19, "inpos_threshold",                 26,  20,   1,  true, "If position error less then threshold INPOS flag is set." },
{  20, "moving_threshold",                27,  21,   1,  true, "If speed is greater then threshold MOVING falg is set." },
{  21, "resistance",                      28,  22,   2,  true, "Resistance value (current controller feedforwad term), voltage ADC units/current ADC units (u4.12)" },
{  22, "speed_filter_alpha",              30,  24,   2,  true, "Speed Kalman filter parameter (alpha coeffeicent of alpha-beta filter), (u0.18)" },
{  23, "speed_filter_beta",               32,  26,   2,  true, "Speed Kalman filter parameter (beta coeffeicent of alpha-beta filter) (u0.20)" },
{  24, "current_kp",                      34,  28,   2,  true, "Current controller proportional gain, current to voltage (u4.12)" },
{  25, "current_ki",                      36,  30,   2,  true, "Current controller integral gain, current to voltage on T1 period (u4.12)." },
{  26, "position_kp",                     38,  32,   2,  true, "Position controller proportional gain, position to current (u8.8)." },
{  27, "position_ki",                     40,  34,   2,  true, "Position controller integral gain, position to current on T2 period (u4.12)." },
{  28, "position_kd",                     42,  36,   2,  true, "Position controller differential gain, speed to current (u18)." },
{  29, "position_Iff0",                   44,  38,   2,  true, "Speed sign feedforward to current, current ADC units (u16)." },
{  30, "position_kff1",                   46,  40,   2,  true, "Speed feedforward, speed to current (u18)" },
{  31, "rt_start_delay",                  48,  42,   1,  true, "Delay between RT_EXCHANGE request and first ACK packet, 10 mcs" },
{  32, "rt_delay",                        49,  43,   1,  true, "Delay between RT_EXCHANGE ACK packets, 10 mcs" },
{  33, "rt_playtime",                     50,  44,   1,  true, "RT_EXCHANGE position exptrapolation duration, 1 mcs" },
{  34, "ack_timeout",                     51,  45,   1,  true, "Maximal allowed ACK delay for all requests except `RT_DEBUG` и `RT_EXCHANGE`." },
{  35, "torque_control",                  -1,  46,   1,  true, "Control mode." },
{  36, "playtime",                        -1,  47,   1,  true, "How long servo attempts to preserve reference speed, T2 period." },
{  37, "position_ref",                    -1,  48,   2,  true, "Reference position, ADC units (s15)." },
{  38, "speed_ref",                       -1,  50,   2,  true, "Reference speed, position ADC unit per T1, (s1.14)." },
{  39, "current_ff",                      -1,  52,   2,  true, "Current feedforward term, reference current or PWM." },
{  40, "position",                        -1,  54,   2, false, "Actual position, ADC units (s15)." },
{  41, "speed",                           -1,  56,   2, false, "Estimated speed, position ADC unit per T1 (s1.14)." },
{  42, "current",                         -1,  58,   2, false, "Estimated current, 0.00148 А (s15)." },
{  43, "pwm_voltage",                     -1,  60,   2, false, "PWM equavalent voltage, 0.0088623 V (s15)." },
{  44, "temperature",                     -1,  62,   1, false, "Temperature readings." },
{  45, "reserved3",                       -1,  63,   1, false, "Reserved" },
{  46, "voltage",                         -1,  64,   2, false, "DC source voltage, 0.0088623 V, u16" },
{  47, "status_error",                    -1,  66,   1, false, "Servo hardware status, see below." },
{  48, "status_detail",                   -1,  67,   1, false, "Servo status, see below." },
{  49, "min_rt_start_delay",              -1,  68,   1, false, "Estimated rt_start_delay value (RT_EXCHANGE request porcessing duration in 10 mcs)." },
{  50, "max_fast_control_delay",          -1,  69,   1, false, "T1 control code execution duration, 1 mcs." },
{  51, "max_slow_control_delay",          -1,  70,   1, false, "T2 control code execution duration, 1 mcs." },
};

const RegisterMapper SweetiBotServoSBS01::register_mapper = RegisterMapper(registers);

SweetiBotServoSBS01::SweetiBotServoSBS01(const std::string& _name, unsigned int _hw_id, bool _reverse, int _offset, double _scale) :
	HerkulexServo(_name, register_mapper, _hw_id, _reverse, _offset, _scale)
{
	max_position = 3996;
	min_position = 100;
};

SweetiBotServoSBS01::SweetiBotServoSBS01(const std::string& _name, unsigned int _hw_id, bool _reverse, int _offset, double _scale, int _min_position, int _max_position) :
	HerkulexServo(_name, register_mapper, _hw_id, _reverse, _offset, _scale, _min_position, _max_position)
{};

double SweetiBotServoSBS01::convertPosRawToRad(unsigned int raw) const 
{
	return scale*POS_CONV_COEFF_RAW2RAD * (static_cast<int16_t>(raw) - offset);
};

unsigned int SweetiBotServoSBS01::convertPosRadToRaw(double pos) const
{
	return pos / (scale*POS_CONV_COEFF_RAW2RAD) + offset;
};

double SweetiBotServoSBS01::convertVelRawToRad(unsigned int raw) const 
{
	return scale * VEL_CONV_COEFF_RAW2RADS * static_cast<int16_t>(raw);
};

unsigned int SweetiBotServoSBS01::convertVelRadToRaw(double vel) const
{
	return vel / (scale*VEL_CONV_COEFF_RAW2RADS);
};

double SweetiBotServoSBS01::convertEffortRawToHm(unsigned int raw) const 
{
	return (EFFORT_CONV_COEFF_RAW2HM/scale) * static_cast<int16_t>(raw);
};

unsigned int SweetiBotServoSBS01::convertEffortHmToRaw(double effort) const
{
	return effort * (scale/EFFORT_CONV_COEFF_RAW2HM);
};

double SweetiBotServoSBS01::convertTimeRawToSec(unsigned int raw) const 
{
	return TIME_CONV_COEFF_RAW2SEC * raw;
};

unsigned int SweetiBotServoSBS01::convertTimeSecToRaw(double time) const
{
	return time / TIME_CONV_COEFF_RAW2SEC;
};

double SweetiBotServoSBS01::convertVoltageRawToVolts(unsigned int raw) const
{
	return raw * VOLTAGE_CONV_COEFF_RAW2VOLT;
}

double SweetiBotServoSBS01::convertTemperatureRawToCelsius(unsigned int raw) const
{
	return 0.0;
}

void SweetiBotServoSBS01::insertRT_EXCHANGEdataConvert(HerkulexPacket& req, double position, double velocity, double effort) const
{
	req.data.push_back(hw_id); // ID
	req.data.push_back(0); // reserved
	// convert to raw
	int position_raw = position / (scale*POS_CONV_COEFF_RAW2RAD) + offset;
	int velocity_raw = velocity / (scale*VEL_CONV_COEFF_RAW2RADS);
	int current_raw = effort * (scale/EFFORT_CONV_COEFF_RAW2HM);
	// add data to packet
	// TODO optimize
	req.data.push_back(position_raw & 0xFF);  
	req.data.push_back((position_raw >> 8) & 0xFF); 
	req.data.push_back(velocity_raw & 0xFF);
	req.data.push_back((velocity_raw >> 8) & 0xFF);
	req.data.push_back(current_raw & 0xFF);
	req.data.push_back((current_raw >> 8) & 0xFF);
}

bool SweetiBotServoSBS01::ackRT_EXCHANGE(const HerkulexPacket& ack, RTState& state) const
{
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RT_EXCHANGE) return false;
	if (ack.data.size() != 8) return false;
	const int16_t * data_as_int16 = (const int16_t *) ack.data.data();
	state.position = (scale*POS_CONV_COEFF_RAW2RAD) * (data_as_int16[0] - offset);
	state.velocity = (scale*VEL_CONV_COEFF_RAW2RADS) * data_as_int16[1];
	state.effort = (EFFORT_CONV_COEFF_RAW2HM/scale) * data_as_int16[2];
	state.temperature = convertTemperatureRawToCelsius(ack.data[6]);
	state.status_error = ack.data[7];
	return true;
}

void SweetiBotServoSBS01::reqPosVel(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 54; // RAW addr of Position TODO: use constexpr
	req.data[1] = 4;
}

bool SweetiBotServoSBS01::ackPosVel(const HerkulexPacket& ack, double& pos, double& vel, Status& status) const 
{
	// read 54 and 55, at addr 58 
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() != 8) return false;
	if (ack.data[0] != 54 || ack.data[1] != 4) return false;
	unsigned int data[2];
	if (!ackRead_impl(ack, 40, data, status)) return false;
	pos = scale*POS_CONV_COEFF_RAW2RAD * ((int16_t) data[0] - offset);
	vel = scale*VEL_CONV_COEFF_RAW2RADS * ((int16_t) data[1]);;
	return true;
}

void SweetiBotServoSBS01::reqPosVelExtended(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 48; // RAW addr of Reference Position
	req.data[1] = 18;
}

bool SweetiBotServoSBS01::ackPosVelExtended(const HerkulexPacket& ack, State& state, Status& status) const
{
	// read 54 throw 60 at addr 60
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() != 18) return false;
	if (ack.data[0] != 48 || ack.data[1] != 14) return false;
	unsigned int data[10];
	if (!ackRead_impl(ack, 37, data, status)) return false;
	state.pos = scale*POS_CONV_COEFF_RAW2RAD * ((int16_t) data[3] - offset);
	state.vel = scale*VEL_CONV_COEFF_RAW2RADS * ((int16_t) data[4]);;
	state.pwm = VOLTAGE_CONV_COEFF_RAW2VOLT * ((int16_t) data[6]);
	state.pos_goal = scale*POS_CONV_COEFF_RAW2RAD * ((int16_t) data[0] - offset);
	state.pos_desired = state.pos_goal;
	state.vel_desired = scale*VEL_CONV_COEFF_RAW2RADS * ((int16_t) data[1]);
	return true;
}

} // namespace servo

} // namespace herkulex 

