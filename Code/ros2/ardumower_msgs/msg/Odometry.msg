#Ardumower odometry data

std_msgs/Header header

# current ticks of each wheel.
# Node Ardumower counts up and down depending on direction.
# if value decreased since last message, it rund backward.

int32 left_ticks
int32 right_ticks