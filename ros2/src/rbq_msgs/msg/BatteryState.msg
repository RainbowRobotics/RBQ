# Status
uint8 STATUS_UNKNOWN = 0
uint8 STATUS_MISSING = 1
uint8 STATUS_CHARGING = 2
uint8 STATUS_DISCHARGING = 3
uint8 STATUS_BOOTING = 4

std_msgs/Header header
string identifier
float64 charge_percentage
float64 current
float64 voltage
float64[] temperatures
uint8 status
