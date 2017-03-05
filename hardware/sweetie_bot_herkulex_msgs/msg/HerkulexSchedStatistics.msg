# Real time exchange statistics for HerkulexSched.
#
# * rt_jog_send_duration --- duration of sendPacket and waitSendPacket call,
# * rt_read_start_time --- RT read round start time from control cycle begining,
# * rt_read_req_duration1 --- duration of first request,
# * rt_read_req_durationN --- duration of last success request,
# * rt_read_n_successes --- number of successful requests,
# * rt_read_n_errors --- number of error,
# * last_erroneous_status --- status of last erroneous packet (error (low)  and detail (high) bytes)
# * rt_cm_start_time --- CM round start time from control cycle begining.
#
float32 rt_jog_send_duration
float32 rt_read_start_time
float32 rt_read_req_duration1
float32 rt_read_req_durationN
uint16 rt_read_n_successes
uint16 rt_read_n_errors
uint16 last_erroneous_status
float32 cm_start_time
