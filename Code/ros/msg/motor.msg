# Ardumower Motor message

Header header

#actual PWM values
int32 leftPWM
int32 rightPWM

# Current in mA
float motorLeftCurrent
float motorRightCurrent

# Power in watts
float motorLeftSense
float motorRightSense

# overload counter
int32 overloadLeft
int32 overloadRight

#Mow motor state
bool mowEnable
float mowCurrent
float mowSense
int32 overloadMow