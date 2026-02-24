# Error report from the STM32, published by the arctos_bridge node.

std_msgs/Header header

# Error codes (matches arctos_error_t enum)
uint8 ERROR_NONE=0
uint8 ERROR_INVALID_AXIS=1
uint8 ERROR_INVALID_PARAM=2
uint8 ERROR_BUSY=3
uint8 ERROR_TRAJ_ID_MISMATCH=4
uint8 ERROR_TRAJ_FULL=5
uint8 ERROR_TRAJ_INCOMPLETE=6
uint8 ERROR_TRAJ_NOT_LOADED=7
uint8 ERROR_ENDSTOP_TRIGGERED=8
uint8 ERROR_TIMEOUT=9
uint8 error_code

string message
