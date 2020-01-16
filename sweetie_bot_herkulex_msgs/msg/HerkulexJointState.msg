# HerkulexJointState.msg
# Deatiled herkulex servo satate.
#
#  * name[] --- servo identificator,
#  * not_responding[] --- servo identificator of not responding servos,
#  * pos[] --- servo position (rad),
#  * vel[] --- servo velocity (rad/s),
#  * pwm[] --- PWM duty ratio,
#  * pos_goal[] --- goal position for trajectory generator,
#  * pos_desired[] --- current reference position,
#  * vel_desired[] --- current reference velocity,
#  * status_error[] --- Herkulex servo status error,
#  * status_detail[] --- Herkulex servo status detail,
#
# Arrays except not_responding must have the same size as name or be empty.
# 
Header header
string[] name
string[] not_responding
float64[] pos
float64[] vel
float64[] pwm
float64[] pos_goal
float64[] pos_desired
float64[] vel_desired
uint8[] status_error
uint8[] status_detail

# command types constants

uint8 STATUS_ERROR_OVER_VOLTAGE = 1
uint8 STATUS_ERROR_POT_LIMIT = 2
uint8 STATUS_ERROR_TEMPERATURE = 4
uint8 STATUS_ERROR_INVALID_PACKET = 8
uint8 STATUS_ERROR_OVERLOAD = 16
uint8 STATUS_ERROR_DRIVER_FAULT = 32
uint8 STATUS_ERROR_EEP_REGS = 64
uint8 STATUS_ERROR_MASK = 119

uint8 STATUS_DETAIL_MOVING = 1
uint8 STATUS_DETAIL_INPOSITION = 2
uint8 STATUS_DETAIL_INVALID_PACKET_CHECKSUM = 4
uint8 STATUS_DETAIL_INVALID_PACKET_UNKNOWN_CMD = 8
uint8 STATUS_DETAIL_INVALID_PACKET_REG_RANGE = 16
uint8 STATUS_DETAIL_INVALID_PACKET_FRAME_ERROR = 32
uint8 STATUS_DETAIL_MOTOR_ON = 64
