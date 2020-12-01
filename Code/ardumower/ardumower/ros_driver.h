// Linux ROS interface
// NOTE: use high baud rates for this serial interface (in mower.h, configure CONSOLE_BAUDRATE to 115200 baud)
#include <ardumower_ros/Status.h>

ros::NodeHandle  nh;

// define messages
ardumower_ros::Status ardumowerStatusMsg;

// Define publisher
ros::Publisher ardumowerStatus("ardumowerStatus", &ardumowerStatusMsg);

void Robot::initROSSerial() {
  // init rosserial to connect to ROS Master
  nh.initNode();
  nh.advertise(ardumowerStatus);
 
}

void Robot::sendROSStatusMessage()
{
  // check if time for ROS message
  if ( millis() >= nextTimeROSStatusMsg ){
  // send ROS status message
  ardumowerStatusMsg.Battery.voltage = batVoltage;
  ardumowerStatusMsg.Battery.charge_voltage = chgVoltage;
  ardumowerStatusMsg.Battery.charge_current = chgCurrent;
  ardumowerStatus.publish( &ardumowerStatusMsg );
  nextTimeROSStatusMsg = millis() + 1000;
  }
  
}


void Robot::spinOnce()
{
  nh.spinOnce();
}
