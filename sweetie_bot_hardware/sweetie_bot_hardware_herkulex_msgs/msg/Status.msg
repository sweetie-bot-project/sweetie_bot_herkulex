# Herkulex servo status.
#
#  * invalid --- message is invalid,
#  * error --- error code,
#  * detail --- status details.
#
uint8 error
uint8 detail
# error flags
uint8 ERROR_OVER_VOLTAGE = 1
uint8 ERROR_POT_LIMIT = 2
uint8 ERROR_TEMPERATURE = 4
uint8 INVALID_PACKET = 8
uint8 ERROR_OVERLOAD = 16
uint8 ERROR_DRIVER_FAULT = 32
uint8 ERROR_EEP_REGS = 64
uint8 ERROR_MASK = 119
# detail flags
uint8 MOVING = 1
uint8 INPOSITION = 2
uint8 INVALID_PACKET_CHECKSUM = 4
uint8 INVALID_PACKET_UNKNOWN_CMD = 8
uint8 INVALID_PACKET_REG_RANGE = 16
uint8 INVALID_PACKET_FRAME_ERROR = 32
uint8 MOTOR_ON = 64
