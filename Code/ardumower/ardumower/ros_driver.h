// Linux ROS interface
// NOTE: use high baud rates for this serial interface (in mower.h, configure CONSOLE_BAUDRATE to 115200 baud)

#ifndef ROS_DRIVER_H
#define ROS_DRIVER_H

#include <ardumower_ros/Status.h>

ros::NodeHandle  nh;

// define messages
ardumower_ros::Status ardumowerStatusMsg;

// Define publisher
ros::Publisher ardumowerStatus("ardumowerStatus", &ardumowerStatusMsg);

void Robot::initROSSerial() {
  // init rosserial to connect to ROS Master
  nh.getHardware()->setBaud(ROS_BAUDRATE);
  nh.initNode();
  nh.advertise(ardumowerStatus);

}

void Robot::sendROSStatusMessage()
{
  // check if time for ROS message
  if ( millis() >= nextTimeROSStatusMsg ) {
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

void Robot::sendROSDebugInfo(int type, char *message)
{
  switch (type)
  {
    case ROS_DEBUG:
      if (ROSDebugVerbose) {
        nh.loginfo(message);
      }
      break;
    case ROS_INFO:
      nh.loginfo(message);
      break;
    case ROS_WARN:
      nh.loginfo(message);
      break;
    case ROS_ERROR:
      nh.loginfo(message);
      break;
    case ROS_FATAL:
      nh.loginfo(message);
      break;
  }
}
/*

  void Robot::sendROSStatusMessage()
  {
  if ( millis() >= nextTimeROSStatusMsg ) {
    Streamprint(s, "$RSTA,%6u,", ((millis() - stateStartTime) / 1000));
    Streamprint(s, "%4s,", stateNames[stateCurr]);
    Streamprint(s, "%4s,", "E000");
    Streamprint(s, "%2d.%01d,", (int)batVoltage, (int)((batVoltage * 10) - ((int)batVoltage * 10)));
    Streamprint(s, "%2d.%01d,", (int)chgVoltage, (int)((chgVoltage * 10) - ((int)chgVoltage * 10)));
    Streamprint(s, "%2d.%01d", (int)chgCurrent, (int)((abs(chgCurrent) * 10) - ((int)abs(chgCurrent) * 10))  );
    Streamprint(s, "\r\n");

    nextTimeROSStatusMsg = millis() + 1000;
  }
  }
*/

#endif
