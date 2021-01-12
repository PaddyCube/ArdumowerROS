# Ardumower Status message
# possible states

int8 STATE_OFF = 0 # off
int8 STATE_ROS = 1 #Linux ROS control
int8 STATE_REMOTE = 2 # model remote control (R/C)
int8 STATE_ERROR = 3 # error
int8 STATE_STATION_CHARGING = 4 # recharge in Station
int8 STATE_STATION = 5 # in station


std_msgs/Header header

string state
int8 state_id
int32 loop_per_sec
