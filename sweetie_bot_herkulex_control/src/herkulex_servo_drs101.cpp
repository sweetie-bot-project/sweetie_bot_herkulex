#include "herkulex_servo_drs101.hpp"


namespace herkulex {

namespace servo {


const unsigned int  HerkulexServoDRS101::POS_RAW_MAX = 1023;

const double HerkulexServoDRS101::POS_CONV_COEFF_RAW2RAD = 0.325*M_PI/180.0;

const double HerkulexServoDRS101::VEL_CONV_COEFF_RAW2RADS = 29.09*M_PI/180.0;

const double HerkulexServoDRS101::TIME_CONV_COEFF_RAW2SEC = 0.0112;

const std::vector<Register> HerkulexServoDRS101::registers_drs101 =
{
//num    name                        eep  ram bytes   rw      description
{ 0,    "reserved0",                 -1,  -1,   0,  false,  "Reserved" },
{ 1,    "model1",                     0,  -1,   1,  false,  "Servo Model No1" },
{ 2,    "model2",                     1,  -1,   1,  false,  "Servo Model No2" },
{ 3,    "version1",                   2,  -1,   1,  false,  "Firmware Version1" },
{ 4,    "version2",                   3,  -1,   1,  false,  "Firmware Version2" },
{ 5,    "baud_rate",                  4,  -1,   1,   true,  "Communication Speed" },
{ 6,    "reserved1",                  5,  -1,   1,  false,  "Reserved" },
{ 7,    "servo_id",                   6,   0,   1,   true,  "Servo ID" },
{ 8,    "ack_policy",                 7,   1,   1,   true,  "ACK Reply policy" },
{ 9,    "alarm_led_policy",           8,   2,   1,   true,  "Blink LED policy" },
{ 10,   "torque_policy",              9,   3,   1,   true,  "Torque policy" },
{ 11,   "reserved2",                  10,  4,   1,  false,  "Reserved" },
{ 12,   "max_temperature",            11,  5,   1,   true,  "Max. temperatue" },
{ 13,   "min_voltage",                12,  6,   1,   true,  "Min. voltage" },
{ 14,   "max_voltage",                13,  7,   1,   true,  "Max. voltage" },
{ 15,   "acceleration_ratio",         14,  8,   1,   true,  "Acceleration ratio" },
{ 16,   "max_acceleration_time",      15,  9,   1,   true,  "Max. acceleration time" },
{ 17,   "dead_zone",                  16, 10,   1,   true,  "Dead zone" },
{ 18,   "saturator_offset",           17, 11,   1,   true,  "Saturator offset" },
{ 19,   "saturator_slope",            18, 12,   2,   true,  "Saturator slope" },
{ 20,   "pwm_offset",                 20, 14,   1,   true,  "PWM Offset" },
{ 21,   "min_pwm",                    21, 15,   1,   true,  "Min. PWM" },
{ 22,   "max_pwm",                    22, 16,   2,   true,  "Max. PWM" },
{ 23,   "overload_pwm_threshold",     24, 18,   2,   true,  "Overload PWM threshold" },
{ 24,   "min_position",               26, 20,   2,   true,  "Min. position" },
{ 25,   "max_position",               28, 22,   2,   true,  "Max. position" },
{ 26,   "position_kp",                30, 24,   2,   true,  "Proportional Gain" },
{ 27,   "position_kd",                32, 26,   2,   true,  "Derivative Gain" },
{ 28,   "position_ki",                34, 28,   2,   true,  "Integral Gain" },
{ 29,   "position_ff_1st_gain",       36, 30,   2,   true,  "Position Feed forward 1st Gain" },
{ 30,   "position_ff_2st_gain",       38, 32,   2,   true,  "Position Feed forward 2st Gain" },
{ 31,   "reserved3",                  40, 34,   2,  false,  "Reserved" },
{ 32,   "reserved4",                  42, 36,   2,  false,  "Reserved" },
{ 33,   "led_blink_period",           44, 38,   1,   true,  "Alarm LED blink period" },
{ 34,   "adc_fault_check_period",     45, 39,   1,   true,  "Temp/voltage error check period" },
{ 35,   "packet_garbage_check_period",46, 40,   1,   true,  "Packet Erorr check period" },
{ 36,   "stop_detection_period",      47, 41,   1,   true,  "Stop detection check period" },
{ 37,   "overload_detection_period",  48, 42,   1,   true,  "Overload Check Interval" },
{ 38,   "stop_threshold",             49, 43,   1,   true,  "Stop Threshold" },
{ 39,   "inposition_margin",          50, 44,   1,   true,  "Offset Threshold" },
{ 40,   "reserved5",                  51, 45,   1,  false,  "Reserved" },
{ 41,   "reserved6",                  52, 46,   1,  false,  "Reserved" },
{ 42,   "calibration_difference",     53, 47,   1,   true,  "Servo Compensation" },
{ 43,   "status_error",               -1, 48,   1,   true,  "Status erorr mask" },
{ 44,   "status_detail",              -1, 49,   1,   true,  "Status detail mask" },
{ 45,   "reserved7",                  -1, 50,   1,  false,  "Reserved" },
{ 46,   "reserved8",                  -1, 51,   1,  false,  "Reserved" },
{ 47,   "torque_control",             -1, 52,   1,   true,  "Torque control" },
{ 48,   "led_control",                -1, 53,   1,   true,  "Led contorl" },
{ 49,   "voltage",                    -1, 54,   1,  false,  "Voltage" },
{ 50,   "temperature",                -1, 55,   1,  false,  "Temperature" },
{ 51,   "current_control_mode",       -1, 56,   1,  false,  "Control mode" },
{ 52,   "tick",                       -1, 57,   1,  false,  "Tick counter" },
{ 53,   "calibrated_position",        -1, 58,   2,  false,  "Calibrated Position" },
{ 54,   "absolute_position",          -1, 60,   2,  false,  "Uncalibrated absolute position" },
{ 55,   "differential_position",      -1, 62,   2,  false,  "Differential Position" },
{ 56,   "pwm",                        -1, 64,   2,  false,  "Current torque in raw data" },
{ 57,   "reserved9",                  -1, 66,   2,  false,  "Reserved" },
{ 58,   "absolute_goal_position",     -1, 68,   2,  false,  "Uncalibrated goal position" },
{ 59,   "absolute_desired_position",  -1, 70,   2,  false,  "Current intermediate goal position in trajectory" },
{ 60,   "desired_velocity",           -1, 72,   2,  false,  "Desired speed" }
};

static const double voltageConversionCoefficient = 0.074075;

// Conversion table for temperature measurement in 10^-2 °C (degreCelsius)
static const short temperatureConversion[] = {-7947,-7178,-6320,-5781,-5380,-5058,-4786,-4549,-4340,-4151,-3979,-3820,-3673,-3535,
	-3406,-3283,-3167,-3057,-2951,-2850,-2753,-2659,-2569,-2482,-2397,-2315,-2236,-2159,-2083,-2010,-1938,-1868,-1800,-1733,-1667,-1603,
	-1539,-1477,-1417,-1357,-1298,-1240,-1183,-1126,-1071,-1016,-962,-909,-856,-804,-753,-702,-652,-602,-553,-504,-456,-408,-361,-314,
	-267,-221,-175,-129,-84,-39,5,49,93,137,181,224,267,310,352,394,437,478,520,562,603,644,686,727,767,808,849,889,929,970,1010,1050,
	1090,1130,1170,1209,1249,1289,1328,1368,1407,1447,1486,1526,1565,1605,1644,1684,1723,1762,1802,1841,1881,1920,1960,1999,2039,2079,
	2119,2158,2198,2238,2278,2318,2359,2399,2439,2480,2520,2561,2602,2643,2684,2725,2766,2808,2850,2891,2933,2976,3018,3060,3103,3146,
	3189,3232,3276,3320,3364,3408,3453,3497,3542,3588,3633,3679,3725,3772,3818,3866,3913,3961,4009,4057,4106,4156,4205,4256,4306,4357,
	4409,4461,4513,4566,4619,4673,4728,4783,4839,4895,4952,5009,5068,5127,5186,5247,5308,5370,5433,5496,5561,5626,5693,5760,5828,5898,
	5968,6040,6113,6187,6263,6339,6417,6497,6578,6861,6746,6832,6920,7010,7102,7196,7292,7391,7492,7596,7703,7812,7925,8041,8160,8284,
	8411,8542,8679,8820,8966,9118,9276,9441,9613,9793,9982,10181,10390,10611,10845,11093,11359,11643,11949,12280,12641,13036,13472,
	13959,14509,15139,15873,16750,17829,19218,21132,24101,30091
};

const RegisterMapper HerkulexServoDRS101::register_mapper_drs101 = RegisterMapper(registers_drs101);

HerkulexServoDRS101::HerkulexServoDRS101(const std::string& _name, unsigned int _hw_id, bool _reverse, int _offset, double _scale) :
	HerkulexServo(_name, register_mapper_drs101, _hw_id, _reverse, _offset, _scale)
{};

double HerkulexServoDRS101::convertVoltageRawToVolts(unsigned int raw) const
{
	return raw * voltageConversionCoefficient;
}

double HerkulexServoDRS101::convertTemperatureRawToCelsius(unsigned int raw) const
{
	return temperatureConversion[raw] / 100.0;
}

double HerkulexServoDRS101::convertVelRawToRad(unsigned int raw) const 
{
	double vel = scale*VEL_CONV_COEFF_RAW2RADS * (int16_t) raw;
	return reverse ? -vel : vel;
};

unsigned int HerkulexServoDRS101::convertVelRadToRaw(double vel) const
{
	vel = reverse ? -vel : vel;
	return vel / (scale*VEL_CONV_COEFF_RAW2RADS);
};

double HerkulexServoDRS101::convertPosRawToRad(unsigned int raw) const 
{
	double pos = scale*POS_CONV_COEFF_RAW2RAD * ((int) raw - offset);
	return reverse ? -pos : pos;
};

unsigned int HerkulexServoDRS101::convertPosRadToRaw(double pos) const
{
	pos = reverse ? -pos : pos;
	return pos / (scale*POS_CONV_COEFF_RAW2RAD) + offset;
};

double HerkulexServoDRS101::convertTimeRawToSec(unsigned int raw) const 
{
	return TIME_CONV_COEFF_RAW2SEC * raw;
};

unsigned int HerkulexServoDRS101::convertTimeSecToRaw(double time) const
{
	return time / TIME_CONV_COEFF_RAW2SEC;
};

void HerkulexServoDRS101::reqStatus(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = u_char(hw_id);
	req.data.resize(2);
	req.data[0] = 55; // RAW addr of Voltage
	req.data[1] = 1;
}

bool HerkulexServoDRS101::ackStatus(const HerkulexPacket& ack, double& temperature, Status& status) const
{
	// read 47 throw 50
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() != 5) return false;
	if (ack.data[0] != 55 || ack.data[1] != 1) return false;
	unsigned int data[1];
	if (!ackRead_impl(ack, 50, data, status)) return false;
	temperature = convertTemperatureRawToCelsius(data[0]);
	return true;
}

void HerkulexServoDRS101::reqStatusExtended(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 52; // RAW addr of Voltage
	req.data[1] = 4;
}

bool HerkulexServoDRS101::ackStatusExtended(const HerkulexPacket& ack, unsigned char& torque_control, unsigned char& led_control, double& voltage, double& temperature, Status& status) const
{
	// read 47 throw 50
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() != 8) return false;
	if (ack.data[0] != 52 || ack.data[1] != 4) return false;
	unsigned int data[4];
	if (!ackRead_impl(ack, 47, data, status)) return false;
	torque_control = u_char(data[0]);
	led_control = u_char(data[1]);
	voltage = convertVoltageRawToVolts(data[2]);
	temperature = convertTemperatureRawToCelsius(data[3]);
	return true;
}

void HerkulexServoDRS101::reqPosVel(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 60; // RAW addr of Absolute Position
	req.data[1] = 4;
}

bool HerkulexServoDRS101::ackPosVel(const HerkulexPacket& ack, double& pos, double& vel, Status& status) const 
{
	// read 54 and 55, at addr 58 
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() != 8) return false;
	if (ack.data[0] != 60 || ack.data[1] != 4) return false;
	unsigned int data[2];
	if (!ackRead_impl(ack, 54, data, status)) return false;
	pos = convertPosRawToRad(data[0]);
	vel = convertVelRawToRad(data[1]);
	return true;
}

void HerkulexServoDRS101::reqPosVelExtended(HerkulexPacket& req) const
{
	req.command = HerkulexPacket::REQ_RAM_READ;
	req.servo_id = hw_id;
	req.data.resize(2);
	req.data[0] = 60; // RAW addr of Absolute Position
	req.data[1] = 14;
}

bool HerkulexServoDRS101::ackPosVelExtended(const HerkulexPacket& ack, State& state, Status& status) const
{
	// read 54 throw 60 at addr 60
	if (ack.servo_id != hw_id) return false;
	if (ack.command != HerkulexPacket::ACK_RAM_READ) return false;
	if (ack.data.size() != 18) return false;
	if (ack.data[0] != 60 || ack.data[1] != 14) return false;
	unsigned int data[7];
	if (!ackRead_impl(ack, 54, data, status)) return false;
	state.pos = convertPosRawToRad(data[0]);
	state.vel = convertVelRawToRad(data[1]);
	state.pwm = float(int16_t(data[2])) / 1023.0f;
	state.pos_goal = convertPosRawToRad(data[4]);;
	state.pos_desired = convertPosRawToRad(data[5]);;
	state.vel_desired = convertVelRawToRad(data[6]);
	return true;
}

} // namespace servo

} // namespace herkulex 

