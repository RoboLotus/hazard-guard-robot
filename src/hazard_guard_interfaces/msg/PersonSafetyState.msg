uint8 CLEAR=0
uint8 CAUTION=1
uint8 SLOW=2
uint8 STOP=3
uint8 SENSOR_FAULT=4

std_msgs/Header header
uint8 state
string state_name
uint32 person_count
float32 nearest_distance_m
bool distance_valid
bool detector_stale
string reason
