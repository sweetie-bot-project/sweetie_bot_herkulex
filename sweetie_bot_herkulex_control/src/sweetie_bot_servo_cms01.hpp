#ifndef  SWEETIE_BOT_SERVO_CMS01_HPP
#define  SWEETIE_BOT_SERVO_CMS01_HPP

#include "herkulex_servo.hpp"

namespace herkulex {

namespace servo {

	class SweetieBotServoCMS01 : public HerkulexServo
	{
		public:
			static const unsigned int POS_RAW_MAX;
			static const double POS_CONV_COEFF_RAW2RAD;
			static const double VEL_CONV_COEFF_RAW2RADS;
			static const double EFFORT_CONV_COEFF_RAW2HM;
			static const double TIME_CONV_COEFF_RAW2SEC;
			static const double VOLTAGE_CONV_COEFF_RAW2VOLT;
			static const std::vector<Register> registers;
			static const RegisterMapper register_mapper;

		public:	
			SweetieBotServoCMS01(const std::string& _name, unsigned int _hw_id, bool _reverse = false, int _offset = POS_RAW_MAX/2, double _scale = 1.0);
			SweetieBotServoCMS01(const std::string& _name, unsigned int _hw_id, bool _reverse, int _offset, double _scale, int _min_position, int _max_position);

			virtual double convertPosRawToRad(unsigned int raw) const;
			virtual unsigned int convertPosRadToRaw(double pos) const;
			virtual double convertVelRawToRad(unsigned int raw) const;
			virtual unsigned int convertVelRadToRaw(double vel) const;
			virtual double convertEffortRawToHm(unsigned int raw) const;
			virtual unsigned int convertEffortHmToRaw(double effort) const;
			virtual double convertTimeRawToSec(unsigned int raw) const;
			virtual unsigned int convertTimeSecToRaw(double pos) const;
			virtual double convertVoltageRawToVolts(unsigned int raw) const;
			virtual double convertTemperatureRawToCelsius(unsigned int raw) const;

			virtual void insertRT_WRITEdataConvert(HerkulexPacket& req, RT_WRITEMode mode, double position, double velocity, double current) const;
			virtual bool ackRT_READ(const HerkulexPacket& ack, RTState& state) const;

			virtual void reqPosVel(HerkulexPacket& req) const;
			virtual bool ackPosVel(const HerkulexPacket& ack, double& pos, double& vel, Status& status) const;
			virtual void reqPosVelExtended(HerkulexPacket& req) const;
			virtual bool ackPosVelExtended(const HerkulexPacket& ack, State& state, Status& status) const;
	};
}

}

#endif  /*SWEETIE_BOT_SERVO_CMS01_HPP*/
