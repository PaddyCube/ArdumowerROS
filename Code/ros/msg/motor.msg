# Ardumower Motor message

Header header

#actual PWM values
int32 leftPWM
int32 rightPWM

# Current in mA
float32 motorLeftCurrent
float32 motorRightCurrent

# Power in watts
float32 motorLeftSense
float32 motorRightSense

# overload counter
int32 overloadLeft
int32 overloadRight

#Mow motor state
bool mowEnable
float32 mowCurrent
float32 mowSense
int32 overloadMow