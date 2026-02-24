# Connection status between the bridge and the STM32.

uint8 DISCONNECTED = 0
uint8 CONNECTED = 1
uint8 RECONNECTING = 2

uint8 status
float64 last_received_time     # seconds since epoch of last broadcast
uint32 reconnect_attempts      # cumulative reconnection attempts
string message
