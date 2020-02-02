# HerkulexState.msg
# Deatiled herkulex servo satate.
#
#  * name --- servo identificator,
#  * respond_sucess --- is servo responded,
#  * torque_control --- torque control mode: 0x40 Break On, 0x60 Torque On, 0x00 Torque Free,
#  * led_control --- led control mode: 0x01 Green, 0x02 Blue, 0x04 Red,
#  * voltage --- voltage in volts,
#  * temperature --- temperature in celsius,
#  * status_error --- Herkulex servo status error,
#  * status_detail --- Herkulex servo status detail,
#
Header header
string name
bool respond_sucess
uint8 torque_control
uint8 led_control
float64 voltage
float64 temperature
uint8 status_error
uint8 status_detail

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

