# Deatiled herkulex servo satate.
#
#  * name --- servo identificator,
#  * pos --- servo position (rad),
#  * vel --- servo velocity (rad/s),
#  * pwm --- PWM duty ratio,
#  * pos_goal --- goal position for trajectory generator,
#  * pos_desired --- current reference position,
#  * vel_desired --- current reference velocity,
#  * status --- Herkulex servo status.
#
# Arrays must have the same size as name or be empty.
# 
Header header
string[] name
float64[] pos
float64[] vel
float64[] pwm
float64[] pos_goal
float64[] pos_desired
float64[] vel_desired
uint8[] error
uint8[] detail

# command types constants

uint8 ERROR_OVER_VOLTAGE = 1
uint8 ERROR_POT_LIMIT = 2
uint8 ERROR_TEMPERATURE = 4
uint8 ERROR_INVALID_PACKET = 8
uint8 ERROR_OVERLOAD = 16
uint8 ERROR_DRIVER_FAULT = 32
uint8 ERROR_EEP_REGS = 64
uint8 ERROR_MASK = 119

uint8 DETAIL_MOVING = 1
uint8 DETAIL_INPOSITION = 2
uint8 DETAIL_INVALID_PACKET_CHECKSUM = 4
uint8 DETAIL_INVALID_PACKET_UNKNOWN_CMD = 8
uint8 DETAIL_INVALID_PACKET_REG_RANGE = 16
uint8 DETAIL_INVALID_PACKET_FRAME_ERROR = 32
uint8 DETAIL_MOTOR_ON = 64
