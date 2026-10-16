# Status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_MISSING = 1
uint8 STATUS_CHARGING = 2
uint8 STATUS_DISCHARGING = 3

std_msgs/Header header
uint32[2] identifier
uint8[2] status
uint8[2] charge_percentage
int8[2] temperature
float32[2] current
float32[2] voltage