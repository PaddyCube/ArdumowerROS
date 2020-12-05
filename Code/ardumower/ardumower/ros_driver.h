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
// NOTE: use high baud rates for this serial interface (in mower.h, configure CONSOLE_BAUDRATE to 115200 baud)

#ifndef ROS_DRIVER_H
#define ROS_DRIVER_H

const char *ROSCommandSet[] = { "$HB", "$RQ", "$RS", "$M1", "$M2", "$EV" };

// ROS commands as enum
enum {
  HEARTBEAT, REQUEST, RESPONSE, MOTORREQUEST, MOTORRESPONSE, EVENT
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

void Robot::readROSSerial() {
  String serialdata;

  if (Console.available() > 0) {
    serialdata = waitStringConsole();
    if (serialdata[0] == '$') {
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
    addErrorCounter (ERR_ROS);
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
          }

      }

    }
  }

}

void Robot::responseHeartBeat() {
  // prepare message
  Console.print("$HB|");
  Console.println(ROSlastMessageID);
}

void Robot::responseStatus() {
  Console.print("$RS|");
  Console.print(ROSlastMessageID);
  Console.print('|');
  Console.print(loopsPerSec);
  Console.print('|');
  Console.print(stateNames[stateCurr]);
  Console.print('|');
  Console.print("E000");
  Console.print('|');
  Console.print(batVoltage);
  Console.print('|');
  Console.print(chgVoltage);
  Console.print('|');
  Console.println(chgCurrent);

}
void Robot::spinOnce() {

}

#endif
