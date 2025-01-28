#include "sweetie_bot_servo_cms01.hpp"

#include <cstdint>

namespace herkulex {

namespace servo {


const unsigned int  SweetieBotServoCMS01::POS_RAW_MAX = 4095;

const double SweetieBotServoCMS01::POS_CONV_COEFF_RAW2RAD = M_PI/180.0*320.0/4096.0;

const double SweetieBotServoCMS01::VEL_CONV_COEFF_RAW2RADS = POS_CONV_COEFF_RAW2RAD * 16000.0 / (1 << 14);

const double SweetieBotServoCMS01::VOLTAGE_CONV_COEFF_RAW2VOLT = 0.0088623;

const double SweetieBotServoCMS01::EFFORT_CONV_COEFF_RAW2HM = 0.00148 * (4.5 / 3.9);

const double SweetieBotServoCMS01::TIME_CONV_COEFF_RAW2SEC = 0.001;

const std::vector<Register> SweetieBotServoCMS01::registers =
{
//num    name                        eep  ram bytes   rw      description
{   0, "model_no_1",                    0,  -1,   1,  false, "Hardware model (major)." },
{   1, "model_no_2",                    1,  -1,   1,  false, "Hardware model (minor)." },
{   2, "version1",                      2,  -1,   1,  false, "Firmware version (major)." },
{   3, "version2",                      3,  -1,   1,  false, "Firmware version (minor)." },
{   4, "baudrate",                      4,  -1,   1,   true, "Baudrate: 0x1 (57600), 0x2 (115200), 0x7 (921600), 0x8 (1000000). See manual for full mode list." },
{   5, "id",                            6,   0,   1,   true, "Servo ID." },
{   6, "ack_policy",                    7,   1,   1,   true, "ACK packet send policy. Flags: { READ_ACK (0x01), WRITE_ACK (0x02), ERROR ACK: (0x04) }" },
{   7, "torque_policy",                 8,   2,   1,   true, "If (r{torque_policy} & r{status_error} & 0x7F) set r{torque_control} to FREE or BRAKE if (r{torque_policy} & 0x80)." },
{   8, "operation_mode",                9,   3,   1,   true, "Control loop configuration. Flags: { CURRENT_CONTROLER_ON (0x01) }" },
{   9, "max_temperature",              10,   4,   1,   true, "Overhead detection threshold." },
{  10, "min_voltage",                  11,   5,   1,   true, "Undervoltage detection threshold." },
{  11, "max_voltage",                  12,   6,   1,   true, "Overvoltage detection threshold." },
{  12, "avg_current_alpha",            13,   7,   1,   true, "Current filter coefficient on T2 period (u.16)." },
{  13, "pwm_deadzone",                 14,   8,   1,   true, "If PWM filling factor is lower than this value apply zero PWM." },
{  14, "max_avg_current",              16,  10,   2,   true, "Average overcurrent detection threshold." },
{  15, "max_current",                  18,  12,   2,   true, "Peak overcurrent detection threshold." },
{  16, "current_limit",                20,  14,   2,   true, "Maximal allowed reference current (saturation limit)." },
{  17, "min_position",                 22,  16,   2,   true, "Movement limit absolute position. Check is performed after applying position offset." },
{  18, "max_position",                 24,  18,   2,   true, "Movement limit absolute position. Check is performed after applying position offset." },
{  19, "inpos_threshold",              26,  20,   1,   true, "If position error less then threshold INPOS flag is set." },
{  20, "moving_threshold",             27,  21,   1,   true, "If speed is greater then threshold MOVING falg is set." },
{  21, "resistance",                   28,  22,   2,   true, "Resistance. This is used to calculate voltage from current reference if current controller is disabled." },
{  22, "inv_back_emf_coeff",           30,  24,   2,   true, "Inversed backEMF coefficient, deg/(sV)." },
{  23, "position_filter_alpha",        32,  26,   2,   true, "Position exponential filter coeffeicent on T1 (u.18)" },
{  24, "speed_filter_alpha",           34,  28,   2,   true, "Speed exponential filter coeffeicent on T1 (u.18)" },
{  25, "current_offset",               36,  30,   2,   true, "Current measurements zero shift. Automatically updated in FREE mode." },
{  26, "current_kp",                   38,  32,   2,   true, "Current controller proportional gain, current to voltage (s3.12)." },
{  27, "current_ki",                   40,  34,   2,   true, "Current controller integral gain, current to voltage on T1 period (s3.12). T1 = T2/16." },
{  28, "current_alpha",                42,  36,   2,   true, "Current controller integral drain (s3.12). If it is nonzero controller compensates backEMF only partially." },
{  29, "position_offset",              44,  38,   2,   true, "Position offset, ADC units (s15)" },
{  30, "position_kp",                  46,  40,   2,   true, "Position controller proportional gain, position to current (u8.8)." },
{  31, "position_ki",                  48,  42,   2,   true, "Position controller np.integral gain, position to current on T2 period (u4.12)." },
{  32, "position_kd",                  50,  44,   2,   true, "Position controller differential gain, speed to current (u.18)." },
{  33, "position_Iff0",                52,  46,   2,   true, "Speed sign feedforward to current, current ADC units (s15)." },
{  34, "position_kff1",                54,  48,   2,   true, "Speed feedforward, speed to current (s.18)" },
{  35, "position_kff2",                56,  50,   2,   true, "Accel feedforward, accel to current (s.32)" },
{  36, "profile_speed",                58,  52,   2,   true, "Maximal speed in accel-speed based speed profile mode." },
{  37, "profile_accel",                60,  54,   2,   true, "Maximal acceleration in accel-speed based speed profile mode." },
{  38, "profile_time_ratio",           62,  56,   1,   true, "Raio beteween duration of acceleration (decelearion) stage and constant-speed stage in time-based profile mode (u.8)." },
{  39, "rt_start_delay",               63,  57,   1,   true, "Delay between RT_READ request and first ACK packet, 10 mcs" },
{  40, "rt_delay",                     64,  58,   1,   true, "Delay between RT_READ ACK packets, 10 mcs" },
{  41, "ack_timeout",                  65,  59,   1,   true, "Maximal allowed ACK packet delay for all requests except `RT_DEBUG` и `RT_EXCHANGE`, 100 mcs." },
{  42, "rt_playtime",                  66,  60,   1,   true, "Position exptrapolation duration after receiving RT_WRITE command, T2 periods." },
{  43, "torque_control",               -1,  61,   1,   true, "Control mode override: FREE (0x00), BRAKE (0x40), NORMAL (Ox60)" },
{  44, "status_error",                 -1,  62,   1,   true, "Servo hardware status." },
{  45, "status_detail",                -1,  63,   1,   true, "Servo status." },
{  46, "position_target",              -1,  64,   2,  false, "Target postion for last PROFILE command (modified by RT_WRITE command)" },
{  47, "playtime",                     -1,  66,   2,  false, "Motion duration for PROFILE_TIME or watchdog time for SPEED, POSITION and CURRENT mode, T2 periods." },
{  48, "position_ref",                 -1,  68,   2,  false, "Reference position set by RT_WRITE (POSITION)  or by trajectory generator (PROFILE, SPEED)." },
{  49, "speed_ref",                    -1,  70,   2,  false, "Reference speed set by RT_WRITE (POSITION, SPEED mode)  or by trajectory generator (PROFILE), position/T1 (s1.14)." },
{  50, "current_ff",                   -1,  72,   2,  false, "Current feedforward modifer set by  RT_WRITE command (POSITION, SPEED, CURRENT)  or by trajectory generator (PROFILE)." },
{  51, "current_ref",                  -1,  74,   2,  false, "Reference current for current regulator." },
{  52, "position_raw",                 -1,  76,   2,  false, "Measured position (u14.2)." },
{  53, "position",                     -1,  78,   2,  false, "Filtered position (s15)." },
{  54, "speed",                        -1,  80,   2,  false, "Estimated speed, position unit per T1 (s1.14)." },
{  55, "current",                      -1,  82,   2,  false, "Actual current." },
{  56, "voltage",                      -1,  84,   2,  false, "DC source voltage." },
{  57, "voltage_pwm",                  -1,  86,   2,  false, "Effective voltage applied to the motor." },
{  58, "temperature",                  -1,  88,   1,  false, "Temperature readings." },
{  59, "control_mode",                 -1,  89,   1,  false, "Current control mode. This value is set by RT_WRITE cmd." },
{  60, "min_rt_start_delay",           -1,  90,   1,  false, "Minimal value of  r{rt_start_delay} value (RT_* request porcessing duration), 10 mcs)." },
{  61, "max_fast_control_delay",       -1,  91,   1,  false, "T1 level control code execution duration, 1 mcs." },
{  62, "max_slow_control_delay",       -1,  92,   1,  false, "T2 level control code execution duration, 1 mcs." },
};


const RegisterMapper SweetieBotServoCMS01::register_mapper = RegisterMapper(registers);

SweetieBotServoCMS01::SweetieBotServoCMS01(const std::string& _name, unsigned int _hw_id, bool _reverse, int _offset, double _scale) :
	HerkulexServo(_name, register_mapper, _hw_id, _reverse, _offset, _scale)
{
	max_position = 3996;
	min_position = 100;
};

SweetieBotServoCMS01::SweetieBotServoCMS01(const std::string& _name, unsigned int _hw_id, bool _reverse, int _offset, double _scale, int _min_position, int _max_position) :
	HerkulexServo(_name, register_mapper, _hw_id, _reverse, _offset, _scale, _min_position, _max_position)
{};

double SweetieBotServoCMS01::convertPosRawToRad(unsigned int raw) const 
{
	return scale*POS_CONV_COEFF_RAW2RAD * (static_cast<int16_t>(raw) - offset);
};

unsigned int SweetieBotServoCMS01::convertPosRadToRaw(double pos) const
{
	return pos / (scale*POS_CONV_COEFF_RAW2RAD) + offset;
};

double SweetieBotServoCMS01::convertVelRawToRad(unsigned int raw) const 
{
	return scale * VEL_CONV_COEFF_RAW2RADS * static_cast<int16_t>(raw);
};

unsigned int SweetieBotServoCMS01::convertVelRadToRaw(double vel) const
{
	return vel / (scale*VEL_CONV_COEFF_RAW2RADS);
};

double SweetieBotServoCMS01::convertEffortRawToHm(unsigned int raw) const 
{
	return (EFFORT_CONV_COEFF_RAW2HM/scale) * static_cast<int16_t>(raw);
};

unsigned int SweetieBotServoCMS01::convertEffortHmToRaw(double effort) const
{
	return effort * (scale/EFFORT_CONV_COEFF_RAW2HM);
};

double SweetieBotServoCMS01::convertTimeRawToSec(unsigned int raw) const 
{
	return TIME_CONV_COEFF_RAW2SEC * raw;
};

unsigned int SweetieBotServoCMS01::convertTimeSecToRaw(double time) const
{
	return time / TIME_CONV_COEFF_RAW2SEC;
};

double SweetieBotServoCMS01::convertVoltageRawToVolts(unsigned int raw) const
{
	return raw * VOLTAGE_CONV_COEFF_RAW2VOLT;
}

double SweetieBotServoCMS01::convertTemperatureRawToCelsius(unsigned int raw) const
{
	return 0.0;
}

void SweetieBotServoCMS01::insertRT_WRITEdataConvert(HerkulexPacket& req, RT_WRITEMode mode,  double position, double velocity, double effort) const
{
	req.data.push_back(hw_id); // ID
	req.data.push_back(static_cast<uint8_t>(mode)); // mode
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

bool SweetieBotServoCMS01::ackRT_READ(const HerkulexPacket& ack, RTState& state) const
{
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RT_READ) return false;
	if (ack.data.size() != 8) return false;
	const int16_t * data_as_int16 = (const int16_t *) ack.data.data();
	state.position = (scale*POS_CONV_COEFF_RAW2RAD) * (data_as_int16[0] - offset);
	state.velocity = (scale*VEL_CONV_COEFF_RAW2RADS) * data_as_int16[1];
	state.effort = (EFFORT_CONV_COEFF_RAW2HM/scale) * data_as_int16[2];
	state.temperature = convertTemperatureRawToCelsius(ack.data[6]);
	state.status_error = ack.data[7];
	return true;
}

void SweetieBotServoCMS01::reqPosVel(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 54; // RAW addr of Position TODO: use constexpr
	req.data[1] = 4;
}

bool SweetieBotServoCMS01::ackPosVel(const HerkulexPacket& ack, double& pos, double& vel, Status& status) const 
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

void SweetieBotServoCMS01::reqPosVelExtended(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 48; // RAW addr of Reference Position
	req.data[1] = 18;
}

bool SweetieBotServoCMS01::ackPosVelExtended(const HerkulexPacket& ack, State& state, Status& status) const
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

