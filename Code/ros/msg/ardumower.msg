# odometry
int odometryLeft
int odometryRight

#mow motor
bool motorMowEnable
float motorMowSense

#wheel motors
float motorLeftSense
float motorRightSense

#Bumpers
int bumperLeftCounter
int bumperRightCounter

#Free Wheel
bool freeWheelIsMoving

# Drop contacts
int dropLeftCounter
int dropRightCounter

#IMU data
float imuDriveHeading
float imuRollHeading

#Perimeter
int perimeterLeftMag
int perimeterRightMag
bool perimeterLeftInside
bool perimeterRightInside

#Sonar
int sonarDistCenter
int sonarDistLeft
int sonarDistRight

#Battery
float batVoltage
float chgVoltage
float chgCurrent

# GPS data
float gpsLat
float gpsLon



#Errors
bool ErrMotorMowOverload
bool ErrMotorLeftOverload
bool ErrMotorRightOverload