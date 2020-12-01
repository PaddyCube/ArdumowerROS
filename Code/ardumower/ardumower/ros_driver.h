// Linux ROS interface
// NOTE: use high baud rates for this serial interface (in mower.h, configure CONSOLE_BAUDRATE to 115200 baud)

ros::NodeHandle  nh;

// define messages
ardumower_msgs::Status ardumowerStatusMsg;

// Define publisher
//ros::Publisher ardumowerStatus("ardumowerStatus", &ardumowerStatus);

void Robot::initROSSerial() {
  // init rosserial to connect to ROS Master
  nh.initNode();
  nh.advertise(ardumowerStatus);
 
}

void Robot::sendROSStatusMessage()
{
  // check if time for ROS message
  // send ROS status message
  ardumowerStatusMsg.data = 0;
  ardumowerStatus.publish( &ardumowerStatusMsg );
  
}


void Robot::spinOnce()
{
  nh.spinOnce();
}
