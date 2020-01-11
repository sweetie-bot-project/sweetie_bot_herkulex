#ifndef HERKULEX_SERVO_DRS101_HPP
#define HERKULEX_SERVO_DRS101_HPP

#include "herkulex_servo.hpp"

namespace herkulex
{

namespace servo
{

class HerkulexServoDRS101 : public HerkulexServo
{
public:
  static const unsigned int POS_RAW_MAX;
  static const double POS_CONV_COEFF_RAW2RAD;
  static const double VEL_CONV_COEFF_RAW2RADS;
  static const double TIME_CONV_COEFF_RAW2SEC;
  static const std::vector<Register> registers_drs101;
  static const RegisterMapper register_mapper_drs101;

public:
  HerkulexServoDRS101(const std::string& _name, unsigned int _hw_id, bool _reverse = false,
                      int _offset = POS_RAW_MAX / 2, double _scale = 1.0);

  virtual double convertVelRawToRad(unsigned int raw) const;
  virtual unsigned int convertVelRadToRaw(double vel) const;
  virtual double convertPosRawToRad(unsigned int raw) const;
  virtual unsigned int convertPosRadToRaw(double pos) const;
  virtual double convertTimeRawToSec(unsigned int raw) const;
  virtual unsigned int convertTimeSecToRaw(double pos) const;

  virtual void reqPosVel(HerkulexPacket& req) const;
  virtual bool ackPosVel(const HerkulexPacket& ack, double& pos, double& vel, Status& status) const;
  virtual void reqState(HerkulexPacket& req) const;
  virtual bool ackState(const HerkulexPacket& ack, State& state, Status& status) const;
};

} // namespace servo

} // namespace herkulex

#endif /*HERKULEX_SERVO_DRS101_HPP*/
