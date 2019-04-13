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
uint16[] status
# command types constants

