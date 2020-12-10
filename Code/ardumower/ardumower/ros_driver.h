// ROS Interface
// The Interface doesn't use rosserial for Arduino as it has some drawbacks like:
// - memory consumption
// - only few publisher / subscriber possible
// - no support for any additional serial console. Using Bluetooth and rosserial at the same time (different serial interfaces) cause crash of Arduino
// - easier to debug as all messages can be monitored using serial console in Arduino IDE
//
//
//  This driver supports all needed components to get commands of the corresponding ROS base controller node.
//  ROS base controller node send commands on regular basis (rate), Arduino respond to this messages with the requested values
//  To detect communication issues, ROS send a message ID at each request and Arduino will use this message ID in response message.
//
// Used commands:
//  $HB Heartbeat (ROS -> Arduino -> ROS)
//  $RQ request sensor data (ROS -> Ardiono)
//  $RS response with requested sensor data (Arduino -> ROS)
//  $M1 Motor command message (ROS -> Arduino)
//  $M2 Motor response message (Arduino -> ROS)
//  $EV Event message (like Bumper or Perimeter hit, Overload etc.) (Arduino -> ROS)
//
//  $LD Debug message to ROS base controller (Arduino -> ROS)
//  $LI Info message to ROS base controller (Arduino -> ROS)
//  $LW Warning message to ROS base controller (Arduino -> ROS)
//  $LE Error message to ROS base controller (Arduino -> ROS)
//  $LF Fatal message to ROS base controller (Arduino -> ROS)
//
//
//
// NOTE: use high baud rates for this serial interface (in mower.h, configure CONSOLE_BAUDRATE to 115200 baud)

#ifndef ROS_DRIVER_H
#define ROS_DRIVER_H

const char *ROSCommandSet[] = { "$HB", "$RQ", "$RS", "$M1", "$M2", "$EV" };

// ROS commands as enum
enum {
  HEARTBEAT, REQUEST, RESPONSE, MOTORREQUEST, MOTORRESPONSE, EVENT
};

// ROS Events
enum {
  ROS_EV_NEW_STATE,
  ROS_EV_SENSOR_TRIGGER,
  ROS_EV_ERROR
};

// ROS sensor requests
enum {
  ROS_SEN_STATUS,
  ROS_SEN_PERIM,
  ROS_SEN_BAT,
  ROS_SEN_MOTOR,
  ROS_SEN_ODOM,
  ROS_SEN_BUMPER,
  ROS_SEN_DROP,
  ROS_SEN_SONAR,
  ROS_SEN_IMU,
  ROS_SEN_RAIN,
  ROS_SEN_FREE_WHEEL,
  ROS_SEN_BUTTON
};
void Robot::initROSSerial() {
  Console.begin(CONSOLE_BAUDRATE);
}

void Robot::sendROSDebugInfo(int type, char *message) {
  switch (type) {
    case ROS_DEBUG:
      if (ROSDebugVerbose) {
        Console.print("$LD:");
        Console.println(message);
      }
      break;
    case ROS_INFO:
      Console.print("$LI:");
      Console.println(message);
      break;
    case ROS_WARN:
      Console.print("$LW:");
      Console.println(message);
      break;
    case ROS_ERROR:
      Console.print("$LE:");
      Console.println(message);
      break;
    case ROS_FATAL:
      Console.print("$LF:");
      Console.println(message);
      break;
  }
}

void Robot::raiseROSNewStateEvent(byte stateNew) {
  Console.print(ROSCommandSet[EVENT]);
  Console.print('|');
  Console.print(ROS_EV_NEW_STATE);
  Console.print('|');
  Console.println(stateNew);
}

void Robot::raiseROSSensorEvent(int sensorType) {
  Console.print(ROSCommandSet[EVENT]);
  Console.print('|');
  Console.print(ROS_EV_SENSOR_TRIGGER);
  Console.print('|');
  Console.println(sensorType);
}

void Robot::raiseROSErrorEvent(byte errorType) {
  Console.print(ROSCommandSet[EVENT]);
  Console.print('|');
  Console.print(ROS_EV_ERROR);
  Console.print('|');
  Console.println(errorType);
}

void Robot::readROSSerial() {
  String serialdata;

  if (Console.available() > 0) {
    serialdata = waitStringConsole();
    if (serialdata[0] == '$') {
      ROSLastTimeMessage = millis();
      processROSCommand(serialdata);
    }
  }

}

void Robot::processROSCommand(String command) {
  // Split command into different sequences
  String commandParts[10];
  String commandData;
  boolean commandPartsFound = true;
  int indexEndOfCommandPart = 0;
  int indexArrayCommandParts = 0;
  unsigned long thisMessageID = 0;
  char commandType[4];

  // Read command
  command.toCharArray(commandType, 4);

  // get payload of command
  commandData = command.substring(4);

  // Split payload by pipe
  while (commandPartsFound == true) {
    indexEndOfCommandPart = commandData.indexOf('|');

    // Check if pipe is found. If not, save the rest of command to array
    if (indexEndOfCommandPart == -1) {
      commandPartsFound = false;
      commandParts[indexArrayCommandParts] = commandData;
    }
    // pipe found, split part of command into array
    else {
      commandParts[indexArrayCommandParts] = commandData.substring(0,
                                             indexEndOfCommandPart);
      commandData = commandData.substring(indexEndOfCommandPart + 1);
      indexArrayCommandParts++;
    }
  }

  // get message ID out of first message sequence
  thisMessageID = atol(commandParts[0].c_str());

  // compare this message ID with the previous once
  if (ROSlastMessageID + 1 != thisMessageID) {
    addErrorCounter(ERR_ROS);
    sendROSDebugInfo(ROS_WARN, "missing Messages");
  }

  ROSlastMessageID = thisMessageID;

  // Check which message needs to be send
  for (int i = 0; i < 6; i++) {
    if (strcmp(commandType, ROSCommandSet[i]) == 0) {

      switch (i) {
        case HEARTBEAT:
          responseHeartBeat();
          break;
        case REQUEST:
          // Check which sensor was requested
          switch (commandParts[1].toInt()) {
            case ROS_SEN_STATUS:
              responseStatus();
              break;
            case ROS_SEN_PERIM:
              responsePerimeter();
              break;
            case ROS_SEN_BAT:
              responseBattery();
              break;
            case ROS_SEN_MOTOR:
              responseMotor();
              break;
            case ROS_SEN_ODOM:
              responseOdometry();
              break;
            case ROS_SEN_BUMPER:
              responseBumper();
              break;
            case ROS_SEN_SONAR:
              responseSonar();
              break;
            case ROS_SEN_BUTTON:
              responseButton();
              break;
            default:
              sendROSDebugInfo(ROS_ERROR, "invalid sensor requested");
              break;
          }
          break;
        case MOTORREQUEST:
          processMotorCommand(commandParts[1], commandParts[2], commandParts[3]);
          break;
      }

    }
  }

}

void Robot::responseHeartBeat() {
  // prepare message
  Console.print(ROSCommandSet[HEARTBEAT]);
  Console.print('|');
  Console.println(ROSlastMessageID);
}

void Robot::responseStatus() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_STATUS);
  Console.print('|');
  Console.print(loopsPerSec);
  Console.print('|');
  Console.print(stateNames[stateCurr]);
  Console.print('|');
  Console.println("E000");

}

void Robot::responseBattery() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_BAT);
  Console.print('|');
  Console.print(batVoltage);
  Console.print('|');
  Console.print(chgVoltage);
  Console.print('|');
  Console.println(chgCurrent);
}

void Robot::responsePerimeter() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_PERIM);
  Console.print('|');
  Console.print(perimeterLeftInside);
  Console.print('|');
  Console.print(perimeterRightInside);
  Console.print('|');
  Console.print(perimeterLeftMag);
  Console.print('|');
  Console.print(perimeterRightMag);
  Console.print('|');
  Console.print(perimeterLastTransitionTime);
  Console.print('|');
  Console.print(perimeter.signalTimedOut(0));
  Console.print('|');
  Console.println(perimeter.signalTimedOut(1));
}

void Robot::responseMotor() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_MOTOR);
  Console.print('|');
  Console.print(motorLeftPWMCurr);
  Console.print('|');
  Console.print(motorRightPWMCurr);
  Console.print('|');
  Console.print(motorLeftSense);  // Power usage in W
  Console.print('|');
  Console.print(motorRightSense);
  Console.print('|');
  Console.print(motorLeftSenseCurrent); // Current in mA
  Console.print('|');
  Console.print(motorRightSenseCurrent);
  Console.print('|');
  Console.print(motorLeftSenseCounter);  // overload counters
  Console.print('|');
  Console.print(motorRightSenseCounter);
  Console.print('|');
  Console.print(motorMowEnable); // mow motor enable
  Console.print('|');
  Console.print(motorMowSense);  // Power in W
  Console.print('|');
  Console.print(motorMowSenseCurrent); // current in mA
  Console.print('|');
  Console.println(motorMowSenseCounter); // overload counter

}

void Robot::responseOdometry() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_ODOM);
  Console.print('|');
  Console.print(odometryLeft);
  Console.print('|');
  Console.println(odometryRight);
}

void Robot::responseBumper() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_BUMPER);
  Console.print('|');
  Console.print(bumperLeftCounter);
  Console.print('|');
  Console.print(bumperRightCounter);
  Console.print('|');
  Console.print(bumperLeft);
  Console.print('|');
  Console.println(bumperRight);
}

void Robot::responseSonar() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(ROS_SEN_SONAR);
  Console.print('|');
  Console.print(sonarDistLeft);
  Console.print('|');
  Console.print(sonarDistCenter);
  Console.print('|');
  Console.print(sonarDistRight);
  Console.print('|');
  Console.println(sonarDistCounter);
}

void Robot::responseButton() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.println(buttonCounter);
  buttonCounter = 0;
}

void Robot::responseMotorCommand() {
  Console.print(ROSCommandSet[MOTORRESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(motorLeftPWMCurr);
  Console.print('|');
  Console.print(motorRightPWMCurr);
  Console.print('|');
  Console.println(motorMowEnable);
}

void Robot::processMotorCommand(String pwmLeftStr, String pwmRightStr, String mowStr)
{
  ROSLastTimeMotorCommand = millis();
  int pwmLeft = pwmLeftStr.toInt();
  int pwmRight = pwmRightStr.toInt();
  int mow = mowStr.toInt();
  bool invalidCommand = false;

  // check for valid commands
  if ( pwmLeft < -255 || pwmLeft > 255 )
  {
    pwmLeft = 0;
    sendROSDebugInfo(ROS_ERROR, "invalid speed for left motor");
    invalidCommand = true;
  }
  if ( pwmRight < -255 || pwmRight > 255)
  {
    pwmRight = 0;
    sendROSDebugInfo(ROS_ERROR, "invalid speed for right motor");
    invalidCommand = true;
  }
  if ( mow < 0 || mow > 1)
  {
    mow = 0;
    sendROSDebugInfo(ROS_ERROR, "invalid value for mow motor");
    invalidCommand = true;
  }
  // set motor speeds accordingly
  if (!invalidCommand)
  {
    setMotorPWM(pwmLeft, pwmRight, false);

    switch (mow)
    {
      case 0:
        motorMowEnable = false;
        break;
      case 1:
        motorMowEnable = true;
        break;
    }
  }
  else {
    setMotorPWM(0, 0, false);
    motorMowEnable = false;
  }


  responseMotorCommand();

}

void Robot::spinOnce() {

}

#endif
