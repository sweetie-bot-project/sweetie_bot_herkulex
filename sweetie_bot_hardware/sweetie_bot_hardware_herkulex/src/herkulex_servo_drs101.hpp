#ifndef  HERKULEX_SERVO_DRS101_HPP
#define  HERKULEX_SERVO_DRS101_HPP

namespace herkulex_servo {

	class HerkulexServoDRS101 {
		public:
			static const double POS_CONV_COEFF_RAW2RAD = 0.325*M_PI/180.0;
			static const double VEL_CONV_COEFF_RAM2RADS = 29.9;
			static const double TIME_CONV_COEFF_RAM2SEC = 0.0115;
			static const std::vector<Register> registers_drs101;
			static const RegisterMapper register_mapper_drs101;
	
		public:	
			HerkulexServo(const std::string _name&, unsigned int _hw_id, bool _reverse, int _offset);

			virtual double convertVelRawToRad(unsigned int raw) const;
			virtual unsigned int convertVelRadToRaw(double vel) const;
			virtual double convertPosRawToRad(unsigned int raw) const;
			virtual unsigned int convertPosRadToRaw(double pos) const;
			virtual double convertTimeRawToSec(unsigned int raw) const;
			virtual unsigned int convertTimeSecToRaw(double pos) const;

			virtual void reqPosVel(HerkulexPacket& req) const;
			virtual bool ackPosVel(HerkulexPacket& ack, double& pos, double& vel);
			virtual void reqState(HerkulexPacket& req) const;
			virtual bool ackState(HerkulexPacket& ack, State& state) const;
	};

};
			


#endif  /*HERKULEX_SERVO_DRS101_HPP*/
