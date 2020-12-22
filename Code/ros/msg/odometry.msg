#Ardumower odometry data

Header header

# current ticks of each wheel.
# Node Ardumower counts up and down depending on direction.
# if value decreased since last message, it rund backward.

int32 leftTicks
int32 rightTicks