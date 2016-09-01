# Real time exchange statistics.
#
# * jog_send_duration --- duration of sendPacket call.
# * avg_request_duration --- average request durattion (request + responce),
# * rt_round_duration --- overall duration of real time round,
# * n_erros --- number of error,
# * n_success --- number of successful requests,
# * last_erroneous_status --- status of last erroneous packet (error (low)  and detail (high) bytes)
#
float32 jog_send_duration
float32 avg_request_duration
float32 rt_round_duration
uint16 n_success
uint16 n_errors
uint16 last_erroneous_status
