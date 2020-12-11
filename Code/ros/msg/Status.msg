# Ardumower Status message
# possible states

int8 STATE_OFF = 0 # off
int8 STATE_ROS = 1 #Linux ROS control
int8 STATE_REMOTE = 2 # model remote control (R/C)
int8 STATE_ERROR = 3 # error
int8 STATE_STATION_CHARGING = 4 # recharge in Station
int8 STATE_STATION = 5 # in station


Header header

string State
int8 StateID
int32 loopPerSec
