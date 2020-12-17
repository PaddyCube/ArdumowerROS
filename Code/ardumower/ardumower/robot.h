/*
  Ardumower (www.ardumower.de)
  Copyright (c) 2013-2014 by Alexander Grau
  Copyright (c) 2013-2014 by Sven Gennat
  Copyright (c) 2014 by Maxime Carpentieri

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

#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#ifdef __AVR__
// Arduino Mega
#include <EEPROM.h>
#else
// Arduino Due
#endif
//#include <Servo.h>  // for RC brushless contoller
#include "drivers.h"
#include "pid.h"
#include "imu.h"

#include "adcman.h"
#include "perimeter.h"
#include "gps.h"
#include "pfod.h"
#include "RunningMedian.h"

/*
  Generic robot class - subclass to implement concrete hardware!

*/
// code version
#define VER "1.0a10-dev Azurit"

// sensors
enum
{
  SEN_STATUS,
  SEN_PERIM_LEFT,        // 0..MAX_PERIMETER
  SEN_PERIM_RIGHT,       // 0..MAX_PERIMETER
  SEN_LAWN_FRONT,
  SEN_LAWN_BACK,
  SEN_BAT_VOLTAGE,  // Volt * 100
  SEN_CHG_CURRENT,  // Ampere * 100
  SEN_CHG_VOLTAGE,  // Volt * 100
  SEN_MOTOR_LEFT,   // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_RIGHT,  // 0..MAX_MOTOR_CURRENT
  SEN_MOTOR_MOW,    // 0..MAX_MOW_CURRENT
  SEN_BUMPER_LEFT,  // LOW = pressed
  SEN_BUMPER_RIGHT, // LOW = pressed
  SEN_DROP_LEFT,    // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
  SEN_DROP_RIGHT,   // LOW = pressed                                                                                                  // Dropsensor - Absturzsensor
  SEN_SONAR_CENTER, // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_LEFT,   // 0..SONAR_TRIGGER_DISTANCE
  SEN_SONAR_RIGHT,  // 0..SONAR_TRIGGER_DISTANCE
  SEN_BUTTON,       // LOW = pressed
  SEN_IMU,
  SEN_ODOM,
  SEN_MOTOR_MOW_RPM,
  SEN_RTC,
  SEN_RAIN,
  SEN_TILT,
  SEN_FREE_WHEEL,
  SEN_NUM_TOKENS  // add this always at the end!!!
};

// actuators
enum
{
  ACT_MOTOR_LEFT,
  ACT_MOTOR_RIGHT,
  ACT_MOTOR_MOW,
  ACT_BUZZER,
  ACT_LED,
  ACT_USER_SW1,
  ACT_USER_SW2,
  ACT_USER_SW3,
  ACT_RTC,
  ACT_CHGRELAY,
  ACT_BATTERY_SW,
};

// error types
enum
{
  ERR_MOTOR_LEFT,
  ERR_MOTOR_RIGHT,
  ERR_MOTOR_MOW,
  ERR_MOW_SENSE,
  ERR_IMU_COMM,
  ERR_IMU_TILT,
  ERR_RTC_COMM,
  ERR_RTC_DATA,
  ERR_PERIMETER_TIMEOUT,
  ERR_ODOMETRY_LEFT,
  ERR_ODOMETRY_RIGHT,
  ERR_BATTERY,
  ERR_CHARGER,
  ERR_GPS_COMM,
  ERR_GPS_DATA,
  ERR_ADC_CALIB,
  ERR_IMU_CALIB,
  ERR_EEPROM_DATA,
  ERR_CPU_SPEED,
  // <---- add new error types here (NOTE: increase MAGIC to avoid corrupt EEPROM error data!)
  ERR_ENUM_COUNT,
  ERR_ROS
};

// finate state machine states
enum
{
  STATE_OFF,     // off
  STATE_ROS,     // Linux ROS control
  STATE_REMOTE,  // model remote control (R/C)
  STATE_ERROR,            // error
  STATE_STATION_CHARGING, // recharge in Station
  STATE_STATION    // in station
};


// console mode
enum
{
  CONSOLE_SENSOR_COUNTERS,
  CONSOLE_SENSOR_VALUES,
  CONSOLE_PERIMETER,
  CONSOLE_OFF
};

// ROS Debug Types
enum
{
  ROS_DEBUG,
  ROS_INFO,
  ROS_WARN,
  ROS_ERROR,
  ROS_FATAL
};

#define MAX_TIMERS 5

#define BATTERY_SW_OFF -1

class Robot
{
  public:
    String name;
    bool developerActive;
    // --------- state machine --------------------------
    byte stateCurr;
    byte stateLast;
    byte stateNext;
    unsigned long stateTime;
    const char *stateName();
    unsigned long stateStartTime;
    unsigned long stateEndTime;
    int idleTimeSec;
    // --------- timer ----------------------------------
    ttimer_t timer[MAX_TIMERS];
    datetime_t datetime;
    char timerUse; // use timer?
    unsigned long nextTimeTimer;
    // ----- bluetooth -------------------------------------
    char bluetoothUse; // use Bluetooth module?
    // ----- esp8266 ---------------------------------------
    char esp8266Use; // use ESP8266 Wifi module?
    String esp8266ConfigString = "";

    // -------- gps state -------------------------------
    GPS gps;
    char gpsUse; // use GPS?
    float gpsLat;
    float gpsLon;
    float gpsX; // X position (m)
    float gpsY; // Y position (m)
    unsigned long nextTimeGPS;
    float stuckIfGpsSpeedBelow;
    int gpsSpeedIgnoreTime; // how long gpsSpeed is ignored when robot switches into a new STATE (in ms)
    int robotIsStuckCounter;
    // -------- odometry state --------------------------
    char twoWayOdometrySensorUse;   // use optional two-wire odometry sensor?
    bool odometryRightSwapDir; // inverse right encoder direction?
    bool odometryLeftSwapDir;  // inverse left encoder direction?
    int odometryLeft;          // left wheel counter
    int odometryRight;         // right wheel counter
    boolean odometryLeftLastState;
    boolean odometryLeftLastState2;
    boolean odometryRightLastState;
    boolean odometryRightLastState2;
    float motorLeftRpmCurr;  // left wheel rpm
    float motorRightRpmCurr; // right wheel rpm
    unsigned long lastMotorRpmTime;
    unsigned long nextTimeOdometry;
    unsigned long nextTimeOdometryInfo;
    boolean odoLeftRightCorrection;
    // -------- RC remote control state -----------------
    char remoteUse;   // use model remote control (R/C)?
    int remoteSteer;  // range -100..100
    int remoteSpeed;  // range -100..100
    int remoteMow;    // range 0..100
    int remoteSwitch; // range 0..100
    unsigned long remoteSteerLastTime;
    unsigned long remoteSpeedLastTime;
    unsigned long remoteMowLastTime;
    unsigned long remoteSwitchLastTime;
    boolean remoteSteerLastState;
    boolean remoteSpeedLastState;
    boolean remoteMowLastState;
    boolean remoteSwitchLastState;
    unsigned long nextTimeRTC;
    // -------- mower motor state -----------------------
    int motorMowRpmCounter; // mower motor speed state
    boolean motorMowRpmLastState;
    boolean motorMowEnable;   // motor can be temporary disabled if stucked etc. with this
    boolean motorMowForceOff; // user switch for mower motor on/off has highest priority
    // --------- wheel motor state ----------------------------
    // wheel motor speed ( <0 backward, >0 forward); range -motorSpeedMaxRpm..motorSpeedMaxRpm
    float motorAccel;           // motor wheel acceleration (warning: do not set too high)
    int motorSpeedMaxRpm;       // motor wheel max RPM
    int motorSpeedMaxPwm;       // motor wheel max Pwm  (8-bit PWM=255, 10-bit PWM=1023)
    float motorPowerMax;        // motor wheel max power (Watt)
    PID motorLeftPID;           // motor left wheel PID controller
    PID motorRightPID;          // motor right wheel PID controller
    float motorSenseRightScale; // motor right sense scale (mA=(ADC-zero)/scale)
    float motorSenseLeftScale;  // motor left sense scale  (mA=(ADC-zero)/scale)
    long motorForwTimeMax; // max. forward time (ms) / timeout
    bool motorRightSwapDir;   // inverse right motor direction?
    bool motorLeftSwapDir;    // inverse left motor direction?
    int motorLeftSpeedRpmSet; // set speed
    int motorRightSpeedRpmSet;
    float motorLeftPWMCurr; // current speed
    float motorRightPWMCurr;
    int motorRightSenseADC;
    int motorLeftSenseADC;
    float motorLeftSenseCurrent;
    float motorRightSenseCurrent;
    float motorLeftSense; // motor power (range 0..MAX_MOTOR_POWER)
    float motorRightSense;
    int motorPowerIgnoreTime;
    int motorZeroSettleTime;   // how long (ms) to wait for motor to settle at zero speed
    int motorLeftSenseCounter; // motor current counter
    int motorRightSenseCounter;
    unsigned long nextTimeMotorSense;
    unsigned long lastSetMotorSpeedTime;
    unsigned long motorLeftZeroTimeout;
    unsigned long motorRightZeroTimeout;
    bool motorErrorSense;
    // -------- mower motor state -----------------------
    // mower motor sppeed; range 0..motorMowSpeedMaxPwm
    float motorMowAccel;      // motor mower acceleration (warning: do not set too high)
    int motorMowSpeedMaxPwm;  // motor mower max PWM
    float motorMowPowerMax;   // motor mower max power (Watt)
    char motorMowModulate;    // motor mower cutter modulation?
    int motorMowRPMSet;       // motor mower RPM (only for cutter modulation)
    float motorMowSenseScale; // motor mower sense scale (mA=(ADC-zero)/scale)
    PID motorMowPID;          // motor mower RPM PID controller
    int motorMowSpeedPWMSet;
    float motorMowPWMCurr; // current speed
    int motorMowSenseADC;
    float motorMowSenseCurrent; // mA
    float motorMowSense;        // motor power (range 0..MAX_MOW_POWER)
    int motorMowSenseCounter;
    int motorMowSenseErrorCounter;
    int motorMowRpmCurr; // motor rpm (range 0..MOW_RPM)
    unsigned long lastMotorMowRpmTime;
    unsigned long nextTimeMotorControl;
    unsigned long nextTimeMotorImuControl;
    unsigned long nextTimeMotorPerimeterControl;
    unsigned long nextTimeMotorMowControl;
    int lastMowSpeedPWM;
    unsigned long lastSetMotorMowSpeedTime;
    unsigned long nextTimeCheckCurrent;
    unsigned long lastTimeMotorMowStuck;
    // --------- BumperDuino state ---------------------------
    // bumper state (true = pressed)
    char bumperUse; // has bumpers?
    char tiltUse;   // has tilt sensor?
    boolean tilt;
    int bumperLeftCounter;
    boolean bumperLeft;
    int bumperRightCounter;
    boolean bumperRight;
    unsigned long nextTimeBumper;
    // --------- free wheel state ---------------------
    boolean freeWheelUse; // has free wheel sensor?
    boolean freeWheelIsMoving;
    unsigned long nextTimeFreeWheel;
    // --------- drop state ---------------------------
    // bumper state (true = pressed)                                                                                                  // Dropsensor - Absturzsensor vorhanden ?
    char dropUse;               // has drops?                                                                                           // Dropsensor - Absturzsensor Zähler links
    int dropLeftCounter;        // Dropsensor - Absturzsensor
    boolean dropLeft;           // Dropsensor - Absturzsensor links betätigt ?
    int dropRightCounter;       // Dropsensor - Absturzsensor
    boolean dropRight;          // Dropsensor - Absturzsensor rechts betätigt ?
    unsigned long nextTimeDrop; // Dropsensor - Absturzsensor
    char dropcontact;           // contact 0-openers 1-closers                                                                                 // Dropsensor Kontakt 0 für Öffner - 1 Schließer
    // ------- IMU state --------------------------------
    IMU imu;
    char imuUse;           // use IMU?
    char imuCorrectDir;    // correct direction by compass?
    PID imuDirPID;         // direction PID controller
    PID imuRollPID;        // roll PID controller
    float imuDriveHeading; // drive heading (IMU)
    float imuRollHeading;  // roll heading  (IMU)
    byte imuRollDir;
    //point_float_t accMin;
    //point_float_t accMax;
    unsigned long nextTimeIMU;       //read IMU data
    unsigned long nextTimeCheckTilt; // check if
    // ------- perimeter state --------------------------
    Perimeter perimeter;
    char perimeterUse; // use perimeter?
    PID perimeterPID;       // perimeter PID controller
    int perimeterLeftMag;       // perimeter magnitude
    int perimeterRightMag;      // perimeter magnitude
    RunningMedian perimeterLeftMagMedian = RunningMedian(300);
    float PeriLeftCoeffAccel;
    RunningMedian perimeterRightMagMedian = RunningMedian(300);
    float PeriRightCoeffAccel;
    int leftSpeedperi;
    int rightSpeedperi;
    unsigned long lastTimeForgetWire;
    int MaxSpeedperiPwm;
    int perimeterLeftMagMaxValue;
    boolean perimeterLeftInside; // is inside perimeter?
    int perimeterRightMagMaxValue;
    boolean perimeterRightInside;       // is inside perimeter?
    unsigned long perimeterTriggerTime; // time to trigger perimeter transition (timeout)
    int perimeterTriggerTimeout;        // perimeter trigger timeout (ms)
    unsigned long perimeterLastTransitionTime;
    int perimeterLeftCounter; // counts perimeter transitions
    unsigned long nextTimePerimeter;
    int perimeterRightCounter; // counts perimeter transitions
    int trackingPerimeterTransitionTimeOut;
    int trackingErrorTimeOut;
    char trackingBlockInnerWheelWhilePerimeterStruggling;
    //  --------- lawn state ----------------------------
    char lawnSensorUse; // use capacitive Sensor
    int lawnSensorCounter;
    boolean lawnSensor;    // lawn capacity sensor state (true = no lawn detected)
    float lawnSensorFront; // front lawn sensor capacity (time)
    float lawnSensorFrontOld;
    float lawnSensorBack; // back lawn sensor capacity (time)
    float lawnSensorBackOld;
    unsigned long nextTimeLawnSensor;
    unsigned long nextTimeLawnSensorCheck;
    // --------- rain -----------------------------------
    boolean rain;
    boolean rainUse;
    int rainCounter;
    unsigned long nextTimeRain;
    // --------- sonar ----------------------------------
    // ultra sonic sensor distance-to-obstacle (cm)
    char sonarUse; // use ultra sonic sensor?
    char sonarLeftUse;
    char sonarRightUse;
    char sonarCenterUse;
    int sonarTriggerBelow; // ultrasonic sensor trigger distance
    int sonarSlowBelow;
    unsigned int sonarDistCenter;
    unsigned int sonarDistRight;
    unsigned int sonarDistLeft;
    unsigned int sonarDistCounter;
    unsigned int tempSonarDistCounter;
    unsigned long sonarObstacleTimeout;
    unsigned long nextTimeSonar;
    unsigned long nextTimeCheckSonar;
    // --------- pfodApp ----------------------------------
    RemoteControl rc; // pfodApp
    unsigned long nextTimePfodLoop;
    // ----- ROS -------------------------------------------
    unsigned long ROSTimeout;
    unsigned long ROSLastTimeMessage;
    unsigned long ROSlastMessageID;
    unsigned long ROSLastTimeMotorCommand;
    unsigned long ROSTimeoutMotorCommand;
    boolean ROSDebugVerbose;
    // general sensor rates
    unsigned int sensorRate[SEN_NUM_TOKENS] = {0};
    unsigned long sensorNextSend[SEN_NUM_TOKENS] = {0};



    // ----- other -----------------------------------------
    char lastSensorTriggered; // last triggered sensor
    unsigned long lastSensorTriggeredTime;
    char buttonUse; // has digital ON/OFF button?
    // ----- user-defined switch ---------------------------
    char userSwitch1; // user-defined switch 1 (default value)
    char userSwitch2; // user-defined switch 2 (default value)
    char userSwitch3; // user-defined switch 3 (default value)
    // --------- charging -------------------------------
    char batMonitor;               // monitor battery and charge voltage?
    float batSwitchOffIfBelow;     // switch off if below voltage (Volt)
    int batSwitchOffIfIdle;        // switch off battery if idle for minutes
    float batFactor;               // battery conversion factor
    float batChgFactor;            // battery conversion factor
    float batFull;                 // battery reference Voltage (fully charged)
    float batChargingCurrentMax;   // maximum current your charger can devliver
    float batFullCurrent;          // current flowing when battery is fully charged
    float startChargingIfBelow;    // start charging if battery Voltage is below
    unsigned long chargingTimeout; // safety timer for charging
    int batADC;
    float chgSenseZero; // charge current sense zero point
    float chgFactor;    // charge current conversion factor
    float chgSense;     // mV/A empfindlichkeit des Ladestromsensors in mV/A (Für ACS712 5A = 185)
    char chgChange;     // messwertumkehr von - nach +         1oder 0
    float batVoltage;   // battery voltage (Volt)
    float DiodeD9;      // Spannungsabfall an der Diode D9 auf den 1.3 Board (Die Spannungsanzeige ist zu niedrig verursacht durch die Diode D9 **UZ**
    byte chgSelection;  // Senor Auswahl
    float batRefFactor;
    float batCapacity;    // battery capacity (mAh)
    float chgVoltage;     // charge voltage (Volt)
    float chgCurrent;     // charge current  (Ampere)
    int chgNull;          // Nulldurchgang Ladestromsensor
    unsigned long nextTimeBattery;
    unsigned long nextTimeCheckBattery;
    int statsBatteryChargingCounter;
    int statsBatteryChargingCounterTotal;
    float statsBatteryChargingCapacityTrip;
    float statsBatteryChargingCapacityTotal;
    float statsBatteryChargingCapacityAverage;
    float lastTimeBatCapacity;
    // --------- error counters --------------------------
    byte errorCounterMax[ERR_ENUM_COUNT]; // maximum error counts seen
    byte errorCounter[ERR_ENUM_COUNT];    // temporary error counts (will be resetted periodically)
    // --------- other ----------------------------------
    int loopsPerSec; // main loops per second
    byte loopsPerSecLowCounter;
    float loopsTa; // main loop-time factor (milliseconds)
    int loopsPerSecCounter;
    byte buttonCounter;
    byte ledState;
    byte consoleMode;
    unsigned long nextTimeButtonCheck;
    unsigned long nextTimeInfo;
    byte rollDir;
    unsigned long nextTimeButton;
    unsigned long nextTimeErrorCounterReset;
    unsigned long nextTimePrintErrors;
    unsigned long nextTimeErrorBeep;
    // ------------robot stats---------------------------
    boolean statsOverride;
    boolean statsMowTimeTotalStart;
    unsigned int statsMowTimeMinutesTripCounter;
    unsigned long statsMowTimeMinutesTotal;
    float statsMowTimeHoursTotal;
    int statsMowTimeMinutesTrip;
    unsigned long nextTimeRobotStats;

    // --------------------------------------------------
    Robot();
    // robot setup
    virtual void setup();
    // robot main loop
    virtual void loop();

    virtual void resetIdleTime();

    // call this from R/C control interrupt
    virtual void setRemotePPMState(unsigned long timeMicros, boolean remoteSpeedState, boolean remoteSteerState,
                                   boolean remoteMowState, boolean remoteSwitchState);
    //Ehl
    // call this from odometry interrupt
    //virtual void setOdometryState(unsigned long timeMicros, boolean odometryLeftState, boolean odometryRightState,
    //  boolean odometryLeftState2, boolean odometryRightState2);
    // call this from hall sensor interrupt
    virtual void setMotorMowRPMState(boolean motorMowRpmState);

    // state machine
    virtual void setNextState(byte stateNew);

    // motor
    virtual void setMotorPWM(int pwmLeft, int pwmRight, boolean useAccel);
    virtual void setMotorMowPWM(int pwm, boolean useAccel);

    // GPS
    virtual void processGPSData();

    // read hardware sensor (HAL)
    virtual int readSensor(char type) {}

    // set hardware actuator (HAL)
    virtual void setActuator(char type, int value) {}

    // settings
    virtual void deleteUserSettings();
    virtual void saveUserSettings();
    virtual void deleteRobotStats();

    // other
    virtual void beep(int numberOfBeeps, boolean shortbeep);
    virtual void printInfo(Stream &s);
    virtual void setUserSwitches();
    virtual void addErrorCounter(byte errType);
    virtual void resetErrorCounters();
    virtual void resetMotorFault() {}
    virtual const char *lastSensorTriggeredName();

  protected:
    // convert ppm time to RC slider value
    virtual int rcValue(int ppmTime);
    virtual void loadSaveErrorCounters(boolean readflag);
    virtual void loadSaveUserSettings(boolean readflag);
    virtual void loadSaveRobotStats(boolean readflag);
    virtual void loadUserSettings();
    virtual void checkErrorCounter();
    virtual void printSettingSerial();

    // read sensors
    virtual void readSensors();

    // read serial
    virtual void readSerial();

    // ROS interface
    virtual void initROSSerial();
    virtual void initROSSensorRates();
    virtual void spinOnce();
    virtual void sendROSDebugInfo(int type, char *);
    virtual void raiseROSNewStateEvent(byte stateNew);
    virtual void raiseROSSensorEvent(int sensortype);
    virtual void raiseROSErrorEvent(byte errorType);
    virtual void readROSSerial();
    virtual void processROSCommand(String command);
    virtual void processMotorCommand(String pwmLeft, String pwmRight, String mow);
    virtual void sendSpinMessage(int sensorID);
    virtual void responseMotorCommand();
    virtual void responseHeartBeat();
    virtual void responseStatus();
    virtual void responsePerimeter();
    virtual void responseMotor();
    virtual void responseOdometry();
    virtual void responseBattery();
    virtual void responseBumper();
    virtual void responseSonar();
    virtual void responseButton();
    virtual void responseIMU();

    // check sensor
    virtual void checkButton();
    virtual void checkBattery();
    //virtual void checkTimer();
    virtual void checkCurrent();
    //virtual void checkBumpers();
    //virtual void checkFreeWheel();
    //virtual void checkDrop(); // Dropsensor - Absturzsensor
    //virtual void checkPerimeterBoundary();
    //virtual void checkLawn();
    virtual void checkSonar();
    virtual void checkTilt();
    // virtual void checkRain();
    virtual void checkTimeout();
    virtual void checkOdometryFaults();
    virtual void checkRobotStats();

    // motor controllers
    virtual void motorControl();
    //  virtual void motorControlImuRoll();
    //  virtual void motorControlPerimeter();
    //  virtual void motorControlImuDir();
    virtual void motorMowControl();

    // date & time
    //virtual void setDefaultTime();

    // set reverse
    //virtual void reverseOrBidir(byte aRollDir);
    //virtual void reverseOrBidirBumper(byte aRollDir);

    // other
    virtual void setSensorTriggered(char type);
    virtual void printRemote();
    //virtual void printOdometry();
    virtual void printMenu();
    virtual void printErrors();
    virtual void delayInfo(int ms);
    virtual void testOdometry();
    virtual void testMotors();
    virtual void testRTC();
    virtual void setDefaults();
    virtual void receiveGPSTime();
    //virtual void calcOdometry();
    virtual void menu();
    virtual void commsMenuBT();
    virtual void commsMenuWifi();
    virtual void commsMenuSelect();
    virtual void configureBluetooth(boolean quick) {};

    // Console helpers
    virtual void purgeConsole();
    virtual char waitCharConsole();
    virtual String waitStringConsole();

    // Spannungsteiler Gesamtspannung ermitteln (Reihenschaltung R1-R2, U2 bekannt, U_GES zu ermitteln)
    virtual float voltageDividerUges(float R1, float R2, float U2);
    // ADC-value to voltage
    virtual float ADC2voltage(float ADCvalue);
};

#endif
