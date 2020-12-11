#!/usr/bin/env python

#ArdumowerROS Driver
# This package contains a class to communicate with Ardumower Hardware
# Package is used to communicate using Serial console and doesn't use any ROS packages at this point

import serial
import time
import rospy
from datetime import datetime

# Ardumower
from ardumower_ros import msg

DEBUG = False

class ArdumowerROSSensors:

    # all types of ROS sensor messages
    SEN_STATUS,SEN_PERIM_LEFT, SEN_PERIM_RIGHT, SEN_LAWN_FRONT,SEN_LAWN_BACK, \
    SEN_BAT_VOLTAGE, SEN_CHG_CURRENT, SEN_CHG_VOLTAGE, \
    SEN_MOTOR_LEFT,SEN_MOTOR_RIGHT,SEN_MOTOR_MOW, \
    SEN_BUMPER_LEFT,SEN_BUMPER_RIGHT,SEN_DROP_LEFT,SEN_DROP_RIGHT, \
    SEN_SONAR_CENTER,SEN_SONAR_LEFT,SEN_SONAR_RIGHT, \
    SEN_BUTTON,SEN_IMU,SEN_ODOM,SEN_MOTOR_MOW_RPM,SEN_RTC,SEN_RAIN,SEN_TILT,SEN_FREE_WHEEL \
     = range(0,26)

    # Error types
    ERR_MOTOR_LEFT,ERR_MOTOR_RIGHT,ERR_MOTOR_MOW,ERR_MOW_SENSE, \
    ERR_IMU_COMM,ERR_IMU_TILT,ERR_RTC_COMM,ERR_RTC_DATA,ERR_PERIMETER_TIMEOUT, \
    ERR_ODOMETRY_LEFT,ERR_ODOMETRY_RIGHT,ERR_BATTERY,ERR_CHARGER,ERR_GPS_COMM, \
    ERR_GPS_DATA,ERR_ADC_CALIB,ERR_IMU_CALIB,ERR_EEPROM_DATA,ERR_CPU_SPEED,ERR_ENUM_COUNT, \
    ERR_ROS = range(0,21)

    # Ardumower States
    STATE_OFF,STATE_ROS,STATE_REMOTE,STATE_ERROR,STATE_STATION_CHARGING, STATE_STATION = range(0,6)

    # Ardumower Event Type
    ROS_EV_NEW_STATE,ROS_EV_SENSOR_TRIGGER,ROS_EV_ERROR = range(0,3)

class ArdumowerROSDriver:
   def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ArdumowerStatus = -1
        self.lastSensorTriggered = -1
        self.lastError = -1
        
        # define publishers here
        self.pubStatus = rospy.Publisher("ardumower_Status",msg.Status, queue_size=10)
        self.pubBattery = rospy.Publisher("ardumower_battery", msg.battery, queue_size=10)
        self.pubBumper = rospy.Publisher("ardumower_bumper", msg.bumper, queue_size=10)
        self.pubPerimeter = rospy.Publisher("ardumower_perimeters", msg.perimeters, queue_size=100)

        self.connect()
        if DEBUG:
            rospy.init_node('ardumower', log_level=rospy.DEBUG)
        else:
            rospy.init_node('ardumower')  

        # ROS Info Messages
        self.ROSMessageID = 0
        self.lastReceivedMessageID = 0

        # ROS poll rates in Hz
        # add here any sensor you want to poll
        self.pollRates = [0] * 26
        self.pollRates.insert(ArdumowerROSSensors.SEN_STATUS, 1) # every 10 sec
        self.pollRates.insert(ArdumowerROSSensors.SEN_BAT_VOLTAGE, 1)   
        self.pollRates.insert(ArdumowerROSSensors.SEN_PERIM_LEFT, 10)

        # time of last poll
        # this list holds the time for the next sensor poll, based on
        # list with ROS poll rates
        self.timeNextPoll = []
        for index, times in enumerate(self.pollRates):
            self.timeNextPoll.insert(index, rospy.get_time() + 10 )

        # ROS Timeouts
        self.timeoutROSMessage = 5 # await at least one message every x sec.
        self.timeLastROSCommand = rospy.get_time() +10 # when last ros command has arrived


    # Method to connect to serial console of Arduino
   def connect(self):
        print "Connecting to Arduino on port", self.port, "..."
        self.port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.timeout)
        # The next line is necessary to give the firmware time to wake up.
        time.sleep(1)
        self.port.flush()
        time.sleep(10)

   # read all incoming messages on serial console
   # analyze kind of message and call corresponding method for further processing
   # check if any message needs to be send to Arduino and trigger command send
   def pollSerial(self):
       while self.port.inWaiting() > 0:
          line = self.port.readline()

          # Check for incoming ROS messages
          if line.startswith('$'):
              # check for command type
               if line.startswith('$LD')  or line.startswith('$LI') or line.startswith('$LW') or line.startswith('$LE') or line.startswith('$LF'):
                 self.processInfoMessage(line)
               elif line.startswith('$RS'):
                   self.processResponseMessage(line)
               elif line.startswith('$EV'):
                   self.processEventMessage(line)


          # Poll sensors only when connected
       if self.ArdumowerStatus != -1:
           
           # Check for poll rates of sensors and others
           currenttime = rospy.get_time()
           # loop through list with poll rates. If poll rate != 0 and
           # corresponding timeout in list timeNextPoll has been exceeded, 
           # request new data for the actual sensor
           for index, polls in enumerate(self.pollRates):
               if currenttime > self.timeNextPoll[index] and polls != 0:
                   self.timeNextPoll[index] = currenttime + 1/float(polls)
                   self.pollSensor(index) 


   # Method process Info messages from Arduino into corresponding
   # ROS Log message
   def processInfoMessage(self, message):
       
       mtype = message[0:4]
       if mtype == '$LD:':
           self.timeLastROSCommand = rospy.get_time()
           self.lastDebugMessage = message[4:]
           rospy.logdebug(message[4:])
       elif mtype == '$LI:':
            self.timeLastROSCommand = rospy.get_time()
            self.lastInfoMessage = message[4:]
            rospy.loginfo(message[4:])
       elif mtype == '$LW:':
            self.timeLastROSCommand = rospy.get_time()
            self.lastWarnMessage = message[4:]
            rospy.logwarn(message[4:])
       elif mtype == '$LE:':
            self.timeLastROSCommand = rospy.get_time()
            self.lastErrorMessage = message[4:]
            rospy.logerr(message[4:])
       elif mtype == '$LF:':
            self.timeLastROSCommand = rospy.get_time()
            self.lastFatalMessage = message[4:]
            rospy.logfatal(message[4:])

   # Methods process any incoming ROS response message (w.o. Info messages)
   # First determine which type has been received. Then call related method for further processing
   def processResponseMessage(self, message):
       self.timeLastROSCommand = rospy.get_time()
       items = message.split("|")
       # get message type
       # check message ID
       self.lastReceivedMessageID = items[1]

       if DEBUG:
           print message

       # Status message
       if items[2] == str(ArdumowerROSSensors.SEN_STATUS):
           msgStatus = msg.Status()
           msgStatus.header.stamp = rospy.Time.now()
           msgStatus.loopPerSec = int(items[3])
           msgStatus.StateID = int(items[4])
           msgStatus.State = items[5]
           self.pubStatus.publish(msgStatus)

       # Battery
       if items[2] == str(ArdumowerROSSensors.SEN_BAT_VOLTAGE) or \
          items[2] == str(ArdumowerROSSensors.SEN_CHG_CURRENT) or \
          items[2] == str(ArdumowerROSSensors.SEN_CHG_VOLTAGE):
           msgBattery = msg.battery()
           msgBattery.header.stamp = rospy.Time.now()
           msgBattery.voltage = float(items[3])
           msgBattery.charge_voltage = float(items[4])
           msgBattery.charge_current = float(items[5])
           self.pubBattery.publish(msgBattery)

       # Bumper
       if items[2] == str(ArdumowerROSSensors.SEN_BUMPER_LEFT) or \
          items[2] == str(ArdumowerROSSensors.SEN_BUMPER_RIGHT):
           msgBumper = msg.bumper()
           msgBumper.header.stamp = rospy.Time.now()
           msgBumper.bumperLeftCount = int(items[3])
           msgBumper.bumperRightCount = int(items[4])
           msgBumper.leftPressed = int(items[5])
           msgBumper.rightPressed = int(items[6])
           self.pubBumper.publish(msgBumper)

       # Perimeter
       if items[2] == str(ArdumowerROSSensors.SEN_PERIM_LEFT) or \
          items[2] == str(ArdumowerROSSensors.SEN_PERIM_RIGHT):
           msgPeriLeft = msg.perimeter()
           msgPeriRight = msg.perimeter()
           msgPeri = msg.perimeters()
           msgPeri.header.stamp = rospy.Time.now()
           msgPeriLeft.inside = int(items[3])
           msgPeriRight.inside = int(items[4])
           msgPeriLeft.magnitude = int(items[5])
           msgPeriRight.magnitude = int(items[6])           
           msgPeriLeft.signal_timeout = int(items[8])
           msgPeriRight.signal_timeout = int(items[9])
           
           msgPeri.data.append(msgPeriLeft)
           msgPeri.data.append(msgPeriRight)
           self.pubPerimeter.publish(msgPeri)           

   # Method process any incoming Event message which has been raised by Ardumower
   def processEventMessage(self, event):

       if DEBUG:
           
           print event
       self.timeLastROSCommand = rospy.get_time()
       items = event.split("|")

       # determine type of event
       if items[1] == str(ArdumowerROSSensors.ROS_EV_NEW_STATE):
           self.ArdumowerStatus = int(items[2])
       
       if items[1] == str(ArdumowerROSSensors.ROS_EV_SENSOR_TRIGGER):
           self.lastSensorTriggered = int(items[2])
           self.pollSensor(self.lastSensorTriggered)
           
    
       if items[1] == str(ArdumowerROSSensors.ROS_EV_ERROR):
           self.lastError = int(items[2])


   # Method to poll a sensor
   # Create command for request string and send it by serial console to Arduino
   def pollSensor(self, sensorID):
       self.ROSMessageID+=1
       cmd = '$RQ|' + str(self.ROSMessageID) + '|' + str(sensorID)+ '\r\n'
       self.port.write(cmd)

   def close(self):
        print "disconnect from serial port"
        self.port.close()


   def spin(self, rate):
       sleeprate = rospy.Rate(rate)
       while not rospy.is_shutdown():
            self.pollSerial()

            # check for timeout
            if rospy.get_time() > self.timeLastROSCommand + self.timeoutROSMessage:
                rospy.logfatal("Message timeout, no messages from Ardumower received")
   
            # check new Ardumower State

            # get data of last triggered sensor
            # self.pollSensor(self.lastSensorTriggered)
            # sleep for rate (Hz))
            sleeprate.sleep()
       self.close()


if __name__ == "__main__": 
    try:
        # Initialize Ardumower ROS Driver   
        robot = ArdumowerROSDriver(port="/dev/ttyACM0", baudrate=115200, timeout=0.5)

        # start observing serial port with given rate
        robot.spin(100)
    except KeyboardInterrupt:
        print "close connection to Ardumower"
        robot.close()