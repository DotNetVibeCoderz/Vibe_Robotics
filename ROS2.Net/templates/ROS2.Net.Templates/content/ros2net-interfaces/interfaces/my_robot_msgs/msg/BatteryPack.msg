# Pack status published on /battery_pack.
std_msgs/Header header
BatteryCell[] cells
float32 state_of_charge 1.0

uint8 STATE_IDLE=0
uint8 STATE_CHARGING=1
uint8 STATE_DISCHARGING=2
uint8 state
