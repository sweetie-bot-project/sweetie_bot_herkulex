uint8 servo_id
uint8 command
uint8[] data
# command types constants
uint8 REQ_EEP_WRITE=1
uint8 REQ_EEP_READ=2
uint8 REQ_RAM_WRITE=3
uint8 REQ_RAM_READ=4
uint8 REQ_I_JOG=5
uint8 REQ_S_JOG=6
uint8 REQ_STAT=7
uint8 REQ_ROLLBACK=8
uint8 REQ_REBOOT=9
uint8 ACK_EEP_WRITE=65
uint8 ACK_EEP_READ=66
uint8 ACK_RAM_WRITE=67
uint8 ACK_RAM_READ=68	
uint8 ACK_I_JOG=69
uint8 ACK_S_JOG=70
uint8 ACK_STAT=71
uint8 ACK_ROLLBACK=72
uint8 ACK_REBOOT=73
uint8 ERR_TIMEOUT=129
uint8 ERR_CHECKSUM=130
# size constants
uint8 DATA_SIZE=213
uint8 HEADER_SIZE=7

