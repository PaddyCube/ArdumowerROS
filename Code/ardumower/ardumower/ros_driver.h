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

//
//SEN_STATUS,
//SEN_PERIM_LEFT,        // 0..MAX_PERIMETER
//SEN_PERIM_RIGHT,       // 0..MAX_PERIMETER
//SEN_LAWN_FRONT,
//SEN_LAWN_BACK,
//SEN_BAT_VOLTAGE,  // Volt * 100
//SEN_CHG_CURRENT,  // Ampere * 100
//SEN_CHG_VOLTAGE,  // Volt * 100
//SEN_MOTOR_LEFT,   // 0..MAX_MOTOR_CURRENT
//SEN_MOTOR_RIGHT,  // 0..MAX_MOTOR_CURRENT
//SEN_MOTOR_MOW,    // 0..MAX_MOW_CURRENT
//SEN_BUMPER_LEFT,  // LOW = pressed
//SEN_BUMPER_RIGHT, // LOW = pressed
//SEN_DROP_LEFT,    // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
//SEN_DROP_RIGHT,   // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
//SEN_SONAR_CENTER, // 0..SONAR_TRIGGER_DISTANCE
//SEN_SONAR_LEFT,   // 0..SONAR_TRIGGER_DISTANCE
//SEN_SONAR_RIGHT,  // 0..SONAR_TRIGGER_DISTANCE
//SEN_BUTTON,       // LOW = pressed
//SEN_IMU,
//SEN_ODOM,
//SEN_MOTOR_MOW_RPM,
//SEN_RTC,
//SEN_RAIN,
//SEN_TILT,
//SEN_FREE_WHEEL
//
//
//
//


void Robot::initROSSensorRates() {

  // add here all sensors which should populate messages at a given rate (ms)
  // other sensors can be triggered by ROS command
  // use this for regular needed messages like Odometry, Perimeter, IMU, Sonar etc
  // don't use this for other sensors like Rain, Button, Bumper to keep
  // communication as low as possible

  sensorRate[SEN_PERIM_LEFT] = 100;
  sensorRate[SEN_BAT_VOLTAGE] = 5000;
  sensorRate[SEN_MOTOR_LEFT] = 100;
  sensorRate[SEN_SONAR_LEFT] = 100;
  sensorRate[SEN_IMU] = 100;
  sensorRate[SEN_ODOM] = 100;     // every 100 ms
  sensorRate[SEN_STATUS] = 1000; // every 10.000ms
}

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
      if (stateCurr != STATE_ROS)
      {
        setNextState(STATE_ROS);
      }
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
            case SEN_STATUS:
              responseStatus();
              break;

            case SEN_PERIM_LEFT:
            case SEN_PERIM_RIGHT:
              responsePerimeter();
              break;

            case SEN_BAT_VOLTAGE:
            case SEN_CHG_VOLTAGE:
            case SEN_CHG_CURRENT:
              responseBattery();
              break;

            case SEN_MOTOR_LEFT:
            case SEN_MOTOR_RIGHT:
            case SEN_MOTOR_MOW:
              responseMotor();
              break;

            case SEN_ODOM:
              responseOdometry();
              break;

            case SEN_BUMPER_LEFT:
            case SEN_BUMPER_RIGHT:
              responseBumper();
              break;

            case SEN_SONAR_CENTER:
            case SEN_SONAR_LEFT:
            case SEN_SONAR_RIGHT:
              responseSonar();
              break;

            case SEN_BUTTON:
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
  Console.print(SEN_STATUS);
  Console.print('|');
  Console.print(loopsPerSec);
  Console.print('|');
  Console.print(stateCurr);
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
  Console.print(SEN_BAT_VOLTAGE);
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
  Console.print(SEN_PERIM_LEFT);
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
  Console.print(SEN_MOTOR_LEFT);
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
  Console.print(SEN_ODOM);
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
  Console.print(SEN_BUMPER_LEFT);
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
  Console.print(SEN_SONAR_CENTER);
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
  Console.print(SEN_BUTTON);
  Console.print('|');
  Console.println(buttonCounter);
  buttonCounter = 0;
}

void Robot::responseIMU() {
  Console.print(ROSCommandSet[RESPONSE]);
  Console.print('|');
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(SEN_IMU);
  Console.print('|');
  Console.print(imu.ypr.yaw / PI * 180);
  Console.print('|');
  Console.print(imu.ypr.pitch / PI * 180);
  Console.print('|');
  Console.print(imu.ypr.roll / PI * 180);
  Console.print('|');
  Console.print(imu.gyro.x / PI * 180);
  Console.print('|');
  Console.print(imu.gyro.y / PI * 180);
  Console.print('|');
  Console.print(imu.gyro.z / PI * 180);
  Console.print('|');
  Console.print(imu.acc.x);
  Console.print('|');
  Console.print(imu.acc.y);
  Console.print('|');
  Console.print(imu.acc.z);
  Console.print('|');
  Console.print(imu.com.x);
  Console.print('|');
  Console.print(imu.com.y);
  Console.print('|');
  Console.println(imu.com.z);
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

  unsigned long now = millis();
  for (int i = 0; i < SEN_NUM_TOKENS; i++)
  {
    if ( ( now > sensorNextSend[i] ) && sensorRate[i] != 0)
    {
      sendSpinMessage(i);
      sensorNextSend[i] = now + sensorRate[i];
    }
  }
}

void Robot::sendSpinMessage(int sensorID) {
  switch (sensorID) {
    case SEN_STATUS:
      responseStatus();
      break;

    case SEN_PERIM_LEFT:
    case SEN_PERIM_RIGHT:
      responsePerimeter();
      break;

    case SEN_BAT_VOLTAGE:
    case SEN_CHG_VOLTAGE:
    case SEN_CHG_CURRENT:
      responseBattery();
      break;

    case SEN_MOTOR_LEFT:
    case SEN_MOTOR_RIGHT:
    case SEN_MOTOR_MOW:
      responseMotor();
      break;

    case SEN_ODOM:
      responseOdometry();
      break;

    case SEN_BUMPER_LEFT:
    case SEN_BUMPER_RIGHT:
      responseBumper();
      break;

    case SEN_SONAR_CENTER:
    case SEN_SONAR_LEFT:
    case SEN_SONAR_RIGHT:
      responseSonar();
      break;

    case SEN_BUTTON:
      responseButton();
      break;

    case SEN_IMU:
      responseIMU();
      break;

    default:
      sendROSDebugInfo(ROS_ERROR, "invalid sensor requested");
      break;
  }
}

#endif
