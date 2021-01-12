# Ardumower Motor message

std_msgs/Header header

#actual PWM values
int32 left_pwm
int32 right_pwm

# Current in mA
float32 motor_left_current
float32 motor_right_current

# Power in watts
float32 motor_left_sense
float32 motor_right_sense

# overload counter
int32 overload_left
int32 overload_right

#Mow motor state
bool mow_enable
float32 mow_current
float32 mow_sense
int32 overload_mow