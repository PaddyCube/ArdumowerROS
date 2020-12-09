/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2015 by Alexander Grau
  Copyright (c) 2013-2015 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri
  Copyright (c) 2014-2015 by Stefan Manteuffel
  Copyright (c) 2015 by Uwe Zimprich
  Private-use only! (you need to ask for a commercial-use)

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)
*/

/* ROS idea:

  In any case we reach an error state, inform ROS by sending a event and stop ROS node. Afterwards, use
  Console.println to display in console what happens.

*/
#include "robot.h"
#include "config.h"
#include "i2c.h"
#include "flashmem.h"

#define MAGIC 52

#define ADDR_USER_SETTINGS 0
#define ADDR_ERR_COUNTERS 400
#define ADDR_ROBOT_STATS 800

const char *stateNames[] = {"OFF ", "ROS", "REMOTE", "ERR ", "CHARG", "STAT"};

const char *sensorNames[] = {"SEN_STATUS", "SEN_PERIM_LEFT", "SEN_PERIM_RIGHT", "SEN_LAWN_FRONT", "SEN_LAWN_BACK",
                             "SEN_BAT_VOLTAGE", "SEN_CHG_CURRENT", "SEN_CHG_VOLTAGE", "SEN_MOTOR_LEFT", "SEN_MOTOR_RIGHT", "SEN_MOTOR_MOW", "SEN_BUMPER_LEFT", "SEN_BUMPER_RIGHT",
                             "SEN_DROP_LEFT", "SEN_DROP_RIGHT", "SEN_SONAR_CENTER", "SEN_SONAR_LEFT", "SEN_SONAR_RIGHT", "SEN_BUTTON", "SEN_IMU", "SEN_MOTOR_MOW_RPM", "SEN_RTC",
                             "SEN_RAIN", "SEN_TILT", "SEN_FREE_WHEEL"
                            };


// --- split robot class ----
#include "battery.h"
#include "consoleui.h"
#include "motor.h"
#include "buzzer.h"
#include "modelrc.h"
#include "settings.h"
#include "ros_driver.h"
// -----------------------------

// Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
float Robot::voltageDividerUges(float R1, float R2, float U2)
{
  return (U2 / R2 * (R1 + R2)); // Uges
}

// ADC-value to voltage
float Robot::ADC2voltage(float ADCvalue)
{
  return (ADCvalue / 1023.0 * IOREF); // ADCman works @ 10 bit
}

Robot::Robot()
{
  name = "Generic";
  developerActive = false;
  rc.setRobot(this);

  lastSensorTriggeredTime = 0;
  stateLast = stateCurr = stateNext = STATE_OFF;
  stateTime = 0;
  idleTimeSec = 0;
  ROSlastMessageID = 0;

  statsMowTimeTotalStart = false;

  odometryLeft = odometryRight = 0;
  odometryLeftLastState = odometryLeftLastState2 = odometryRightLastState = odometryRightLastState2 = LOW;

  motorRightRpmCurr = motorLeftRpmCurr = 0;
  lastMotorRpmTime = 0;
  lastSetMotorSpeedTime = 0;
  motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
  motorLeftPWMCurr = motorRightPWMCurr = 0;
  motorRightSenseADC = motorLeftSenseADC = 0;
  motorLeftSenseCurrent = motorRightSenseCurrent = 0;
  motorLeftSense = motorRightSense = 0;
  motorLeftSenseCounter = motorRightSenseCounter = 0;
  motorZeroSettleTime = 0;
  motorLeftZeroTimeout = 0;
  motorRightZeroTimeout = 0;

  remoteSteer = remoteSpeed = remoteMow = remoteSwitch = 0;
  remoteSteerLastTime = remoteSpeedLastTime = remoteMowLastTime = remoteSwitchLastTime = 0;
  remoteSteerLastState = remoteSpeedLastState = remoteMowLastState = remoteSwitchLastState = LOW;

  motorMowRpmCounter = 0;
  motorMowRpmLastState = LOW;
  motorMowEnable = false;
  motorMowForceOff = false;
  motorMowSpeedPWMSet = motorSpeedMaxRpm;
  motorMowPWMCurr = 0;
  motorMowSenseADC = 0;
  motorMowSenseCurrent = 0;
  motorMowSense = 0;
  motorMowSenseCounter = 0;
  motorMowSenseErrorCounter = 0;
  motorMowRpmCurr = 0;
  lastMowSpeedPWM = 0;
  lastSetMotorMowSpeedTime = 0;
  nextTimeCheckCurrent = 0;
  lastTimeMotorMowStuck = 0;

  bumperLeftCounter = bumperRightCounter = 0;
  bumperLeft = bumperRight = false;

  dropLeftCounter = dropRightCounter = 0; // Dropsensor - Absturzsensor
  dropLeft = dropRight = false;           // Dropsensor - Absturzsensor

  gpsLat = gpsLon = gpsX = gpsY = 0;
  robotIsStuckCounter = 0;

  imuDriveHeading = 0;
  imuRollHeading = 0;

  perimeterLeftMag = 1;
  perimeterRightMag = 1;
  perimeterLeftMagMedian.add(perimeterLeftMag);
  perimeterLeftMagMedian.add(perimeterRightMag);
  perimeterLeftInside = true;
  perimeterRightInside = true;
  perimeterLeftCounter = 0;
  perimeterRightCounter = 0;
  perimeterLastTransitionTime = 0;
  perimeterTriggerTime = 0;

  lawnSensorCounter = 0;
  lawnSensor = false;
  lawnSensorFront = lawnSensorFrontOld = lawnSensorBack = lawnSensorBackOld = 0;

  rain = false;
  rainCounter = 0;

  freeWheelIsMoving = false;

  sonarLeftUse = sonarRightUse = sonarCenterUse = false;
  sonarDistCenter = sonarDistRight = sonarDistLeft = 0;
  sonarDistCounter = 0;
  tempSonarDistCounter = 0;
  sonarObstacleTimeout = 0;

  batADC = 0;
  batVoltage = 0;
  batRefFactor = 0;
  batCapacity = 0;
  lastTimeBatCapacity = 0;
  chgVoltage = 0;
  chgCurrent = 0;

  memset(errorCounterMax, 0, sizeof errorCounterMax);
  memset(errorCounter, 0, sizeof errorCounterMax);

  loopsPerSec = 0;
  loopsPerSecLowCounter = 0;
  loopsTa = 5.0;
  loopsPerSecCounter = 0;
  buttonCounter = 0;
  ledState = 0;

  nextTimeButtonCheck = 0;
  nextTimeInfo = 0;
  nextTimeMotorSense = 0;
  nextTimeIMU = 0;
  nextTimeCheckTilt = 0;
  nextTimeOdometry = 0;
  nextTimeOdometryInfo = 0;
  nextTimeBumper = 0;
  nextTimeDrop = 0; // Dropsensor - Absturzsensor
  nextTimeSonar = 0;
  nextTimeFreeWheel = 0;
  nextTimeBattery = 0;
  nextTimeCheckBattery = millis() + 10000;
  nextTimePerimeter = 0;
  nextTimeLawnSensor = 0;
  nextTimeLawnSensorCheck = 0;
  nextTimePrintErrors = 0;
  nextTimeTimer = millis() + 60000;
  nextTimeRTC = 0;
  nextTimeGPS = 0;
  nextTimePfodLoop = 0;
  nextTimeRain = 0;
  lastMotorMowRpmTime = millis();
  nextTimeButton = 0;
  nextTimeErrorCounterReset = 0;
  nextTimeErrorBeep = 0;
  nextTimeMotorControl = 0;
  nextTimeMotorImuControl = 0;
  nextTimeMotorPerimeterControl = 0;
  nextTimeMotorMowControl = 0;

  nextTimeRobotStats = 0;
  statsMowTimeMinutesTripCounter = 0;
  statsBatteryChargingCounter = 0;
}

void Robot::setSensorTriggered(char type)
{
  lastSensorTriggered = type;
  lastSensorTriggeredTime = millis();

  // ROS event raise here
  raiseROSSensorEvent(type);

}

const char *Robot::lastSensorTriggeredName()
{
  String s = "";
  if (lastSensorTriggeredTime != 0)
  {
    s = sensorNames[lastSensorTriggered];
    s += " (";
    s += String((millis() - lastSensorTriggeredTime) / 1000);
    s += " s ago)";
  }
  return s.c_str();
}

void Robot::resetIdleTime()
{
  if (idleTimeSec == BATTERY_SW_OFF)
  { // battery switched off?
    Console.println(F("BATTERY switching ON again"));
    setActuator(ACT_BATTERY_SW, 1); // switch on battery again (if connected via USB)
  }
  idleTimeSec = 0;
}

void Robot::beep(int numberOfBeeps, boolean shortbeep = false)
{
  for (int i = 0; i < numberOfBeeps; i++)
  {
    setActuator(ACT_BUZZER, 4200);
    if (shortbeep)
      delay(50);
    else
      delay(500);
    setActuator(ACT_BUZZER, 0);
    if (shortbeep)
      delay(250);
    else
      delay(500);
  }
}

// set user-defined switches
void Robot::setUserSwitches()
{
  setActuator(ACT_USER_SW1, userSwitch1);
  setActuator(ACT_USER_SW2, userSwitch2);
  setActuator(ACT_USER_SW3, userSwitch3);
}

void Robot::setup()
{
  //setDefaultTime();
  setMotorPWM(0, 0, false);
  loadSaveErrorCounters(true);
  loadUserSettings();
  if (!statsOverride)
    loadSaveRobotStats(true);
  else
    loadSaveRobotStats(false);
  setUserSwitches();
  if (!ADCMan.calibrationDataAvail())
  {
    ADCMan.calibrate();
  }

  if (!buttonUse)
  {
    // robot has no ON/OFF button => start immediately
    // ROS
    // setNextState(STATE_FORWARD,0);
  }

  stateStartTime = millis();
  beep(1);
  initROSSerial(); // start serial console
  sendROSDebugInfo(ROS_DEBUG, "SETUP");

  Console.println(F("-------------------START-------------------"));
  sendROSDebugInfo(ROS_INFO, "Ardumower ROS");
#if defined(PCB_1_2)
  sendROSDebugInfo(ROS_DEBUG, "PCB 1_2");
#elif defined(PCB_1_3)
  sendROSDebugInfo(ROS_DEBUG, "PCB 1_3");
#endif
#ifdef __AVR__
  // Console.print(F("  Arduino Mega"));
  sendROSDebugInfo(ROS_DEBUG, "Arduino Mega");
#else
  // Console.print(F("  Arduino Due"));
  sendROSDebugInfo(ROS_DEBUG, "Arduino Due");
#endif
  Console.print(F("  IOREF="));
  Console.println(IOREF);

  Console.print(F("Robot: "));
  Console.println(name);

  Console.println(F("press..."));
  Console.println(F("  d main menu"));
  Console.println(F("  l simulate left bumper"));
  Console.println(F("  r simulate right bumper"));
  Console.println();
  Console.println(F("-------------------------------------------"));

  // ROS read serial console here before starting ROS nodes

  delay(5000);
  readSerial();
  //rc.readSerial();
  //resetIdleTime();
  
  Console.println("Init ROSSerial");
  raiseROSNewStateEvent(stateCurr); // ready for communication
  ROSLastTimeMessage = millis();
}

void Robot::checkButton()
{
  if ((!buttonUse) || (millis() < nextTimeButtonCheck))
    return;

  nextTimeButtonCheck = millis() + 50;
  boolean buttonPressed = (readSensor(SEN_BUTTON) == LOW);
  if (((!buttonPressed) && (buttonCounter > 0)) || ((buttonPressed) && (millis() >= nextTimeButton)))
  {
    nextTimeButton = millis() + 1000;
    if (buttonPressed)
    {
      sendROSDebugInfo(ROS_DEBUG, "buttonPressed");
      //Console.println(F("buttonPressed"));
      // ON/OFF button pressed
      beep(1);
      buttonCounter++;
      // setSensorTriggered(SEN_BUTTON);
      // if (!rmcsUse)
      //{
      resetIdleTime();
      // }
    }
    else
    {
      setSensorTriggered(SEN_BUTTON);
    }

    //  buttonCounter = 0;
  }
}


void Robot::readSensors()
{
  //NOTE: this function should only read in sensors into variables - it should NOT change any state!

  if (millis() >= nextTimeMotorSense)
  {
    nextTimeMotorSense = millis() + 50;
    double accel = 0.05;
    motorRightSenseADC = readSensor(SEN_MOTOR_RIGHT);
    motorLeftSenseADC = readSensor(SEN_MOTOR_LEFT);
    motorMowSenseADC = readSensor(SEN_MOTOR_MOW);

    motorRightSenseCurrent = motorRightSenseCurrent * (1.0 - accel) + ((double)motorRightSenseADC) * motorSenseRightScale * accel;
    motorLeftSenseCurrent = motorLeftSenseCurrent * (1.0 - accel) + ((double)motorLeftSenseADC) * motorSenseLeftScale * accel;
    // NOTE for motor mower current : we double motor current as two drivers are connected in parallel
    motorMowSenseCurrent = motorMowSenseCurrent * (1.0 - accel) + ((double)motorMowSenseADC) * motorMowSenseScale * accel * 2;

    if (batVoltage > 8)
    {
      motorRightSense = motorRightSenseCurrent * batVoltage / 1000; // conversion to power in Watt
      motorLeftSense = motorLeftSenseCurrent * batVoltage / 1000;
      motorMowSense = motorMowSenseCurrent * batVoltage / 1000;
    }
    else
    {
      motorRightSense = motorRightSenseCurrent * batFull / 1000; // conversion to power in Watt in absence of battery voltage measurement
      motorLeftSense = motorLeftSenseCurrent * batFull / 1000;
      motorMowSense = motorMowSenseCurrent * batFull / 1000;
    }

    if ((millis() - lastMotorMowRpmTime) >= 500)
    {
      motorMowRpmCurr = readSensor(SEN_MOTOR_MOW_RPM);
      if ((motorMowRpmCurr == 0) && (motorMowRpmCounter != 0))
      {
        // rpm may be updated via interrupt
        motorMowRpmCurr = (int)((((double)motorMowRpmCounter) / ((double)(millis() - lastMotorMowRpmTime))) * 60000.0);
        motorMowRpmCounter = 0;
      }
      lastMotorMowRpmTime = millis();
      if (!ADCMan.calibrationDataAvail())
      {
        //Console.println(F("Error: missing ADC calibration data"));
        addErrorCounter(ERR_ADC_CALIB);
        setNextState(STATE_ERROR);
        sendROSDebugInfo(ROS_FATAL, "missing ADC calibration data");
      }
    }
  }

  // ROS add second perimeter coil here
  if ((perimeterUse) && (millis() >= nextTimePerimeter))
  {
    // if (stateCurr == STATE_PERI_TRACK)
    nextTimePerimeter = millis() + 30;
    // else
    //  nextTimePerimeter = millis() + 50; // 50
    perimeterLeftMag = readSensor(SEN_PERIM_LEFT);
    perimeterRightMag = readSensor(SEN_PERIM_RIGHT);
    //    if (stateCurr == STATE_PERI_FIND){
    //      perimeterLeftMagMedian.add(abs(perimeterLeftMag));
    //      perimeterLeftMagMedian.add(abs(perimeterLeftMag));
    //    }
    if ((perimeter.isInside(0) != perimeterLeftInside))
    {
      perimeterLeftCounter++;
      setSensorTriggered(SEN_PERIM_LEFT);
      perimeterLastTransitionTime = millis();
      perimeterLeftInside = perimeter.isInside(0);
    }
    if ((perimeter.isInside(1) != perimeterRightInside))
    {
      perimeterRightCounter++;
      setSensorTriggered(SEN_PERIM_RIGHT);
      perimeterLastTransitionTime = millis();
      perimeterRightInside = perimeter.isInside(1);
    }
    static boolean LEDstate = false;
    if (perimeterLeftInside && perimeterRightInside && !LEDstate)
    {
      setActuator(ACT_LED, HIGH);
      LEDstate = true;
    }
    if ( (!perimeterLeftInside || !perimeterRightInside) && LEDstate)
    {
      setActuator(ACT_LED, LOW);
      LEDstate = false;
    }
    if ((!perimeterLeftInside || !perimeterRightInside) && (perimeterTriggerTime == 0))
    {
      // set perimeter trigger time
      if (millis() > stateStartTime + 2000)
      { // far away from perimeter?
        perimeterTriggerTime = millis() + perimeterTriggerTimeout;
      }
      else
      {
        perimeterTriggerTime = millis();
      }
    }
    if (perimeter.signalTimedOut(0) || perimeter.signalTimedOut(1))
    {

      //   Console.println("Error: perimeter too far away");
      // ROS raise event perimeter error
      sendROSDebugInfo(ROS_FATAL, "perimeter too far away");
      addErrorCounter(ERR_PERIMETER_TIMEOUT);
      setNextState(STATE_ERROR);

    }
  }

  if ((lawnSensorUse) && (millis() >= nextTimeLawnSensor))
  {
    nextTimeLawnSensor = millis() + 100;
    double accel = 0.03;
    lawnSensorFront = (1.0 - accel) * lawnSensorFront + accel * ((double)readSensor(SEN_LAWN_FRONT));
    lawnSensorBack = (1.0 - accel) * lawnSensorBack + accel * ((double)readSensor(SEN_LAWN_BACK));
  }
  if ((lawnSensorUse) && (millis() >= nextTimeLawnSensorCheck))
  {
    nextTimeLawnSensorCheck = millis() + 2000;
    double deltaFront = lawnSensorFront / lawnSensorFrontOld * 100.0;
    double deltaBack = lawnSensorBack / lawnSensorBackOld * 100.0;
    if ((deltaFront <= 95) || (deltaBack <= 95))
    {
      /*   Console.print(F("LAWN "));
        Console.print(deltaFront);
        Console.print(",");
        Console.println(deltaBack); */
      lawnSensorCounter++;
      setSensorTriggered(SEN_LAWN_FRONT);
      lawnSensor = true;
    }
    lawnSensorFrontOld = lawnSensorFront;
    lawnSensorBackOld = lawnSensorBack;
  }

  if ((sonarUse) && (millis() >= nextTimeSonar))
  {
    static char senSonarTurn = SEN_SONAR_CENTER;
    nextTimeSonar = millis() + 250;

    switch (senSonarTurn)
    {
      case SEN_SONAR_RIGHT:
        if (sonarRightUse)
          sonarDistRight = readSensor(SEN_SONAR_RIGHT);
        senSonarTurn = SEN_SONAR_LEFT;
        break;
      case SEN_SONAR_LEFT:
        if (sonarLeftUse)
          sonarDistLeft = readSensor(SEN_SONAR_LEFT);
        senSonarTurn = SEN_SONAR_CENTER;
        break;
      case SEN_SONAR_CENTER:
        if (sonarCenterUse)
          sonarDistCenter = readSensor(SEN_SONAR_CENTER);
        senSonarTurn = SEN_SONAR_RIGHT;
        break;
      default:
        senSonarTurn = SEN_SONAR_CENTER;
        break;
    }
  }

  if ((freeWheelUse) && (millis() >= nextTimeFreeWheel))
  {
    nextTimeFreeWheel = millis() + 100;
    freeWheelIsMoving = (readSensor(SEN_FREE_WHEEL) == 0);
    if (!freeWheelIsMoving)
      setSensorTriggered(SEN_FREE_WHEEL);
  }

  if ((bumperUse) && (millis() >= nextTimeBumper))
  {
    nextTimeBumper = millis() + 100;
    tilt = (readSensor(SEN_TILT) == 0);

    if (readSensor(SEN_BUMPER_LEFT) == 0)
    {
      bumperLeftCounter++;
      setSensorTriggered(SEN_BUMPER_LEFT);
      bumperLeft = true;
    }
    else {
      bumperLeft = false; // Bumper released
    }

    if (readSensor(SEN_BUMPER_RIGHT) == 0)
    {
      bumperRightCounter++;
      setSensorTriggered(SEN_BUMPER_RIGHT);
      bumperRight = true;
    }
    else {
      bumperRight = false;
    }
  }

  if ((dropUse) && (millis() >= nextTimeDrop))
  { // Dropsensor - Absturzsensor
    nextTimeDrop = millis() + 100; // Dropsensor - Absturzsensor
    if (readSensor(SEN_DROP_LEFT) == dropcontact)
    { // Dropsensor - Absturzsensor
      dropLeftCounter++; // Dropsensor - Absturzsensor
      setSensorTriggered(SEN_DROP_LEFT);
      dropLeft = true; // Dropsensor - Absturzsensor
    }                  // Dropsensor - Absturzsensor
    else {
      dropLeft = false;
    }

    if (readSensor(SEN_DROP_RIGHT) == dropcontact)
    { // Dropsensor - Absturzsensor
      dropRightCounter++; // Dropsensor - Absturzsensor
      setSensorTriggered(SEN_DROP_RIGHT);
      dropRight = true; // Dropsensor - Absturzsensor
    }
    else {
      dropRight = false;
    }
  }

  if ((timerUse) && (millis() >= nextTimeRTC))
  {
    nextTimeRTC = millis() + 60000;
    readSensor(SEN_RTC); // read RTC
  }

  if ((imuUse) && (millis() >= nextTimeIMU))
  {
    // IMU
    readSensor(SEN_IMU);
    nextTimeIMU = millis() + 200; // 5 hz
    if (imu.getErrorCounter() > 0)
    {
      addErrorCounter(ERR_IMU_COMM);
      sendROSDebugInfo(ROS_FATAL, "IMU comm error");
      // ROS raise event IMU error
      //Console.println(F("IMU comm error"));
    }
    if (!imu.calibrationAvail)
    {
      // ROS raise event IMU error
      // Console.println(F("Error: missing IMU calibration data"));
      sendROSDebugInfo(ROS_FATAL, "missing IMU calibration data");
      addErrorCounter(ERR_IMU_CALIB);
      setNextState(STATE_ERROR);
    }
  }

  if (millis() >= nextTimeBattery)
  {
    // read battery
    nextTimeBattery = millis() + 100;
    if ((abs(chgCurrent) > 0.04) && (chgVoltage > 5))
    {
      // charging
      batCapacity += (chgCurrent / 36.0);
    }
    // convert to double
    batADC = readSensor(SEN_BAT_VOLTAGE);
    int currentADC = readSensor(SEN_CHG_CURRENT);
    int chgADC = readSensor(SEN_CHG_VOLTAGE);

    double batvolt = ((double)batADC) * batFactor / 10;     // / 10 due to arduremote bug, can be removed after fixing
    double chgvolt = ((double)chgADC) * batChgFactor / 10;  // / 10 due to arduremote bug, can be removed after fixing
    double curramp = ((double)currentADC) * chgFactor / 10; // / 10 due to arduremote bug, can be removed after fixing

#if defined(PCB_1_3)             // Prüfe ob das V1.3 Board verwendet wird - und wenn ja **UZ**
    batvolt = batvolt + DiodeD9; // dann rechnet zur Batteriespannung den Spannungsabfall der Diode D9 hinzu. (Spannungsabfall an der Diode D9 auf den 1.3 Board (Die Spannungsanzeige ist zu niedrig verursacht durch die Diode D9) **UZ**
#endif                           // **UZ**

    // low-pass filter
    double accel = 0.01;
    //double accel = 1.0;
    if (abs(batVoltage - batvolt) > 5)
      batVoltage = batvolt;
    else
      batVoltage = (1.0 - accel) * batVoltage + accel * batvolt;
    if (abs(chgVoltage - chgvolt) > 5)
      chgVoltage = chgvolt;
    else
      chgVoltage = (1.0 - accel) * chgVoltage + accel * chgvolt;
    if (abs(chgCurrent - curramp) > 0.5)
      chgCurrent = curramp;
    else
      chgCurrent = (1.0 - accel) * chgCurrent + accel * curramp;
  }

  if ((rainUse) && (millis() >= nextTimeRain))
  {
    // read rain sensor
    nextTimeRain = millis() + 5000;
    rain = (readSensor(SEN_RAIN) != 0);
    if (rain)
    {
      rainCounter++;
      setSensorTriggered(SEN_RAIN);
    }
  }
}

void Robot::setDefaults()
{
  motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
  motorMowEnable = false;
}

void Robot::receiveGPSTime()
{
  if (gpsUse)
  {
    unsigned long chars = 0;
    unsigned short good_sentences = 0;
    unsigned short failed_cs = 0;
    gps.stats(&chars, &good_sentences, &failed_cs);
    if (good_sentences == 0)
    {
      // no GPS sentences received so far
      // ROS raise error GPS
      // Console.println(F("GPS communication error!"));
      addErrorCounter(ERR_GPS_COMM);
      // next line commented out as GPS communication may not be available if GPS signal is poor
      //setNextState(STATE_ERROR, 0);
    }
    // ROS send GPS data
    /* Console.print(F("GPS sentences: "));
      Console.println(good_sentences);
      Console.print(F("GPS satellites in view: "));
      Console.println(gps.satellites());   */
    if (gps.satellites() == 255)
    {
      // no GPS satellites received so far
      addErrorCounter(ERR_GPS_DATA);
    }
    int year;
    byte month, day, hour, minute, second, hundredths;
    unsigned long age;
    gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    if (age != GPS::GPS_INVALID_AGE)
    {
      // ROS send GPS date/time
      //Console.print(F("GPS date received: "));
      //Console.println(date2str(datetime.date));
      datetime.date.dayOfWeek = getDayOfWeek(month, day, year, 1);
      datetime.date.day = day;
      datetime.date.month = month;
      datetime.date.year = year;
      datetime.time.hour = hour;
      datetime.time.minute = minute;
      if (timerUse)
      {
        // set RTC using GPS data
        //Console.print(F("RTC date set: "));
        //Console.println(date2str(datetime.date));
        setActuator(ACT_RTC, 0);
      }
    }
  }
}

// check motor current
void Robot::checkCurrent()
{
  if (millis() < nextTimeCheckCurrent)
    return;
  nextTimeCheckCurrent = millis() + 100;

  //bb add test MotorCurrent in manual mode and stop immediatly If >Powermax
  //  if (stateCurr == STATE_MANUAL)
  //  {
  //    if (motorLeftSense >= motorPowerMax)
  //    {
  //      motorLeftSenseCounter++;
  //      setSensorTriggered(SEN_MOTOR_LEFT);
  //      setMotorPWM(0, 0, false);
  //      addErrorCounter(ERR_MOTOR_LEFT);
  //      setNextState(STATE_ERROR, 0);
  //      // ROS raise error event
  //      // Console.println("Error: Motor Left current");
  //    }
  //    if (motorRightSense >= motorPowerMax)
  //    {
  //      motorRightSenseCounter++;
  //      setSensorTriggered(SEN_MOTOR_RIGHT);
  //      setMotorPWM(0, 0, false);
  //      addErrorCounter(ERR_MOTOR_RIGHT);
  //      setNextState(STATE_ERROR, 0);
  //      // ROS raise error event
  //      //Console.println("Error: Motor Right current");
  //    }
  //  }

  if (motorMowSense >= motorMowPowerMax)
  {
    motorMowSenseCounter++;
    setSensorTriggered(SEN_MOTOR_MOW);
  }
  else
  {
    errorCounterMax[ERR_MOW_SENSE] = 0;
    motorMowSenseCounter = 0;
    if ((lastTimeMotorMowStuck != 0) && (millis() >= lastTimeMotorMowStuck + 30000))
    { // wait 30 seconds before switching on again
      errorCounter[ERR_MOW_SENSE] = 0;
      motorMowEnable = true;
      lastTimeMotorMowStuck = 0;
    }
  }

  if (motorMowSenseCounter >= 30)
  { //ignore motorMowPower for 3 seconds
    motorMowEnable = false;
    // ROS raise error event
    // Console.println("Error: Motor mow current");
    addErrorCounter(ERR_MOW_SENSE);
    lastTimeMotorMowStuck = millis();
  }

  if (motorLeftSense >= motorPowerMax)
  {
    // left wheel motor overpowered
    // ROS here we need to swap some States
    if ((millis() > stateStartTime + motorPowerIgnoreTime))
    {
      //beep(1);
      motorLeftSenseCounter++;
      setSensorTriggered(SEN_MOTOR_LEFT);
      setMotorPWM(0, 0, false);
    }

  }
  else if (motorRightSense >= motorPowerMax)
  {
    // right wheel motor overpowered
    // ROS here we need to swap some stages again
    if ( millis() > stateStartTime + motorPowerIgnoreTime)
    {
      //beep(1);
      motorRightSenseCounter++;
      setSensorTriggered(SEN_MOTOR_RIGHT);
      setMotorPWM(0, 0, false);
    }
  }
}
//
//// check bumpers
//void Robot::checkBumpers()
//{
//  if (!bumperUse)
//    return;
//
//  if ((bumperLeft || bumperRight))
//  {
//    // ROS raise
//  }
//}
//
//// check free wheel
//void Robot::checkFreeWheel()
//{
//  if (!freeWheelUse)
//    return;
//
//  if (!freeWheelIsMoving)
//  {
//
//    // ROS raise event
//  }
//}
//
//// check drop                                                                                                                       // Dropsensor - Absturzsensor
//void Robot::checkDrop()
//{ // Dropsensor - Absturzsensor
//  if (!dropUse)
//    return;
//
//  if ((dropLeft || dropRight))
//  { // Dropsensor - Absturzsensor
//
//    // ROS raise event                                                                                                                      // Dropsensor - Absturzsensor
//  } // Dropsensor - Absturzsensor
//} // Dropsensor - Absturzsensor

//
//// check perimeter as a boundary
//void Robot::checkPerimeterBoundary()
//{
//  if (!perimeterUse)
//    return;
//  if (!perimeterLeftInside || !perimeterRightInside)
//  {
//    // ROS raise event
//
//  }
//}

//// check lawn
//void Robot::checkLawn()
//{
//  if (!lawnSensorUse)
//    return;
//  if ((lawnSensor))
//  {
//    // ROS raise event
//  }
//  else
//    lawnSensor = false;
//}
//
//void Robot::checkRain()
//{
//  if (!rainUse)
//    return;
//  if (rain)
//  {
//    // ROS raise event
//
//  }
//}

// check sonar
void Robot::checkSonar()
{
  if (!sonarUse)
    return;
  if (millis() < nextTimeCheckSonar)
    return;
  nextTimeCheckSonar = millis() + 500;
  if (millis() < stateStartTime + 4000)
    return;
  if (sonarDistCenter < 11 || sonarDistCenter > 100)
    sonarDistCenter = NO_ECHO; // Objekt ist zu nah am Sensor Wert ist unbrauchbar
  if (sonarDistRight < 11 || sonarDistRight > 100)
    sonarDistRight = NO_ECHO; // Object is too close to the sensor. Sensor value is useless
  if (sonarDistLeft < 11 || sonarDistLeft > 100)
    sonarDistLeft = NO_ECHO; // Filters spiks under the possible detection limit

  // slow down motor wheel speed near obstacles
  //  if ((stateCurr == STATE_FORWARD)) //|| (((stateCurr == STATE_FORWARD) || (stateCurr == STATE_REVERSE))))
  //  {
  //    if (sonarObstacleTimeout == 0)
  //    {
  //      if (((NO_ECHO != sonarDistCenter) && (sonarDistCenter < sonarSlowBelow)) || ((NO_ECHO != sonarDistRight) && (sonarDistRight < sonarSlowBelow)) || ((NO_ECHO != sonarDistLeft) && (sonarDistLeft < sonarSlowBelow)))
  //      {
  //        tempSonarDistCounter++;
  //        if (tempSonarDistCounter >= 5)
  //        {
  //          // Console.println("sonar slow down");
  //          // ROS raise event obstacle close
  //          sonarObstacleTimeout = millis() + 3000;
  //        }
  //      }
  //      else
  //        tempSonarDistCounter = 0;
  //    }
  //    else if ((sonarObstacleTimeout != 0) && (millis() > sonarObstacleTimeout))
  //    {
  //      //Console.println("no sonar");
  //      sonarObstacleTimeout = 0;
  //      tempSonarDistCounter = 0;
  //// no more close obstacle
  //    }
  //  }

  if (sonarTriggerBelow != 0)
  {
    if ((sonarDistCenter != NO_ECHO) && (sonarDistCenter < sonarTriggerBelow))
    {
      sonarDistCounter++;
      setSensorTriggered(SEN_SONAR_CENTER);
    }
    if ((sonarDistRight != NO_ECHO) && (sonarDistRight < sonarTriggerBelow))
    {
      sonarDistCounter++;
      setSensorTriggered(SEN_SONAR_RIGHT);
    }
    if ((sonarDistLeft != NO_ECHO) && (sonarDistLeft < sonarTriggerBelow))
    {
      sonarDistCounter++;
      setSensorTriggered(SEN_SONAR_LEFT);
    }
  }
}

// check BumperDuino tilt, IMU tilt
void Robot::checkTilt()
{
  if (millis() < nextTimeCheckTilt)
    return;
  nextTimeCheckTilt = millis() + 200; // 5Hz same as nextTimeImu

  if (tiltUse)
  {
    if ((tilt))
    {
      // Console.println(F("BumperDuino tilt"));
      setSensorTriggered(SEN_TILT);
      // ROS raise event tilt
    }
  }

  if (!imuUse)
    return;
  int pitchAngle = (imu.ypr.pitch / PI * 180.0);
  int rollAngle = (imu.ypr.roll / PI * 180.0);
  // if ((stateCurr != STATE_OFF) && (stateCurr != STATE_ERROR) && (stateCurr != STATE_STATION))
  // {
  if ((abs(pitchAngle) > 40) || (abs(rollAngle) > 40))
  {
    //  Console.println(F("Error: IMU tilt"));
    addErrorCounter(ERR_IMU_TILT);
    setSensorTriggered(SEN_TILT);
    setNextState(STATE_ERROR);
  }
  //  }

}


void Robot::processGPSData()
{
  if (millis() < nextTimeGPS)
    return;
  nextTimeGPS = millis() + 1000;
  float nlat, nlon;
  unsigned long age;
  gps.f_get_position(&nlat, &nlon, &age);
  if (nlat == GPS::GPS_INVALID_F_ANGLE)
    return;
  if (gpsLon == 0)
  {
    gpsLon = nlon; // this is xy (0,0)
    gpsLat = nlat;
    return;
  }
  gpsX = (float)gps.distance_between(nlat, gpsLon, gpsLat, gpsLon);
  gpsY = (float)gps.distance_between(gpsLat, nlon, gpsLat, gpsLon);
}
// ROS Timeout Check
void Robot::checkTimeout()
{
  // Check for last ROs message and raise timeout to stop entire robot
  // ROS raise event lane timeout

}

const char *Robot::stateName()
{
  return stateNames[stateCurr];
}

void Robot::setNextState(byte stateNew) {

  if (stateCurr == STATE_STATION_CHARGING) {
    // always switch off charging relay if leaving state STATE_STATION_CHARGING
    setActuator(ACT_CHGRELAY, 0);
  }

  if (stateNew == STATE_STATION) {
    setMotorPWM(0, 0, false);
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();
    statsMowTimeTotalStart = false;  // stop stats mowTime counter
    loadSaveRobotStats(false);        //save robot stats

  }
  if (stateNew == STATE_STATION_CHARGING) {
    setActuator(ACT_CHGRELAY, 1);
    setDefaults();
  }
  if (stateNew == STATE_OFF) {
    setActuator(ACT_CHGRELAY, 0);
    setDefaults();
    statsMowTimeTotalStart = false; // stop stats mowTime counter
    loadSaveRobotStats(false);      //save robot stats
  }
  if (stateNew == STATE_ERROR) {
    motorMowEnable = false;
    motorLeftSpeedRpmSet = motorRightSpeedRpmSet = 0;
    setActuator(ACT_CHGRELAY, 0);
    statsMowTimeTotalStart = false;
    //loadSaveRobotStats(false);
  }
  // Inform ROS about new state
  raiseROSNewStateEvent(stateNew);
  stateCurr = stateNew;
  stateStartTime = millis();
  stateLast = stateCurr;

}

// HERE WAS STATE MACHINE, NOT NEEDED AS ROS TAKES ANY DECISION

void Robot::loop()
{
  stateTime = millis() - stateStartTime;
  int steer;
  ADCMan.run();

  // ROS no read of serial console in loop, only setup
  readROSSerial();
  rc.readSerial();
  //resetIdleTime();

  readSensors();
  checkBattery();
  checkRobotStats();
  //  calcOdometry();
  //   checkOdometryFaults();
  checkButton();
  // motorMowControl();
  checkTilt();

  if (imuUse)
    imu.update();

  if (gpsUse)
  {
    gps.feed();
    processGPSData();
  }

  if (millis() >= nextTimePfodLoop)
  {
    nextTimePfodLoop = millis() + 200;
    rc.run();
  }

  if (millis() >= nextTimeInfo)
  {
    nextTimeInfo = millis() + 1000;

    // ROS send info for debugging
    ledState = ~ledState;
    //checkErrorCounter();
    sendROSDebugInfo(ROS_DEBUG, "alive");

    if (stateCurr == STATE_REMOTE) {
      //   printRemote();
    }

    loopsPerSec = loopsPerSecCounter;

    if (stateCurr != STATE_ERROR)
    {
      if (loopsPerSec < 10)
      { // loopsPerSec too low
        if (loopsPerSecLowCounter < 255)
          loopsPerSecLowCounter++;
      }
      else if (loopsPerSecLowCounter > 0)
        loopsPerSecLowCounter--; // loopsPerSec OK
      if (loopsPerSecLowCounter > 10)
      { // too long I2C cables can be a reason for this
        // Console.println(F("Error: loopsPerSec too low (check I2C cables)"));
        // ROS raise event Arduino too slow
        addErrorCounter(ERR_CPU_SPEED);
        setNextState(STATE_ERROR); //mower is switched into ERROR
      }
    }
    else
      loopsPerSecLowCounter = 0; // reset counter to zero
    if (loopsPerSec > 0)
      loopsTa = 1000.0 / ((double)loopsPerSec);
    loopsPerSecCounter = 0;
  }

  // Process ROS commands here

  // check if ROS timeout occured
  if ( (millis() - ROSLastTimeMessage > ROSTimeout ) && stateCurr != STATE_ERROR )
  {
    Console.println("ROS Timeout");
    addErrorCounter(ERR_ROS);
    setNextState(STATE_ERROR);
  }

  // state machine - things to do *PERMANENTLY* for current state
  // robot state machine
  // http://wiki.ardumower.de/images/f/ff/Ardumower_states.png
  switch (stateCurr)
  {
    case STATE_ERROR:
      // fatal-error
      if (millis() >= nextTimeErrorBeep)
      {
        nextTimeErrorBeep = millis() + 5000;
        beep(1, true);
      }
      break;
    case STATE_OFF:
      // robot is turned off
      //checkTimer();   // deactivated due to safety issues. when mower is off it should stay off. timer is only active when mower is n STATE_STATION.
      if (batMonitor && (millis() - stateStartTime > 2000))
      {
        if (chgVoltage > 5.0)
        {
          beep(2, true);
          setNextState(STATE_STATION);

        }
      }
      imuDriveHeading = imu.ypr.yaw;
      break;

    case STATE_STATION:
      // waiting until auto-start by user or timer triggered
      if (batMonitor) {
        if (chgVoltage > 5.0) {
          if (batVoltage < startChargingIfBelow && (millis() - stateStartTime > 2000)) {
            setNextState(STATE_STATION_CHARGING);
          }
        } else setNextState(STATE_OFF);
      }
      break;

    case STATE_STATION_CHARGING:
      // waiting until charging completed
      if (batMonitor) {
        if ((chgCurrent < batFullCurrent) && (millis() - stateStartTime > 2000)) setNextState(STATE_STATION);
        else if (millis() - stateStartTime > chargingTimeout) {
          Console.println(F("Battery chargingTimeout"));
          addErrorCounter(ERR_BATTERY);
          setNextState(STATE_ERROR);
        }
        if (chgVoltage < 5 && (millis() - stateStartTime > 2000)) setNextState(STATE_OFF);
      }
      break;
      //  case STATE_REMOTE:
      //    // remote control mode (RC)
      //    //if (remoteSwitch > 50) setNextState(STATE_FORWARD, 0);
      //    steer = ((double)motorSpeedMaxRpm / 2) * (((double)remoteSteer) / 100.0);
      //    if (remoteSpeed < 0)
      //      steer *= -1;
      //    motorLeftSpeedRpmSet = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed) / 100.0) - steer;
      //    motorRightSpeedRpmSet = ((double)motorSpeedMaxRpm) * (((double)remoteSpeed) / 100.0) + steer;
      //    motorLeftSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorLeftSpeedRpmSet));
      //    motorRightSpeedRpmSet = max(-motorSpeedMaxRpm, min(motorSpeedMaxRpm, motorRightSpeedRpmSet));
      //    motorMowSpeedPWMSet = ((double)motorMowSpeedMaxPwm) * (((double)remoteMow) / 100.0);
      //    break;


  } // end switch

  // next line deactivated (issue with RC failsafe)
  //if ((useRemoteRC) && (remoteSwitch < -50)) setNextState(STATE_REMOTE, 0);

  // decide which motor control to use
  // ROS redefine needed

 // motorControl();
//if (stateCurr != STATE_REMOTE)
 //   motorMowSpeedPWMSet = motorMowSpeedMaxPwm;

  //  bumperRight = false;
  //  bumperLeft = false;
  //
  //  dropRight = false; // Dropsensor - Absturzsensor
  //  dropLeft = false;  // Dropsensor - Absturzsensor

  loopsPerSecCounter++;
}
