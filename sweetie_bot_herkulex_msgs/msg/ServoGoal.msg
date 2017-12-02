# Goal definition for position controlled servo with trapezoidal speed profile.
#
#  * name --- servo identification string.
#  * target_pos --- target positions (rad).
#  * playtime --- movement duration (s).
#
# Arrays sould have the same sizes. Only exception is playtime, which can alse contains one value 
# if movement duration is equal for all servos. 
#
# For better performance of Herkulex DRS101 servos select playtime equal to n*11.5 ms, 
# where n is integer.
# 
string[] name
float64[] target_pos
float64[] playtime
