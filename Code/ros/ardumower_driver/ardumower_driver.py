#!/usr/bin/env python

#ArdumowerROS Driver
# This package contains a class to communicate with Ardumower Hardware
# Package is used to communicate using Serial console and doesn't use any ROS packages at this point

import serial
import time
import rospy
from datetime import datetime

DEBUG = False

class ArdumowerROSSensors:

    # all types of ROS sensor messages
    ROS_SEN_STATUS,ROS_SEN_PERIM,ROS_SEN_BAT,ROS_SEN_MOTOR,ROS_SEN_ODOM,ROS_SEN_BUMPER,ROS_SEN_DROP,ROS_SEN_SONAR,ROS_SEN_IMU,ROS_SEN_RAIN,ROS_SEN_FREE_WHEEL = range(0,11)

class ArdumowerROSDriver:
   def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5):
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

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
        self.pollRates = [0] * 20
        self.pollRates.insert(ArdumowerROSSensors.ROS_SEN_STATUS, 0.1) # every 10 sec
        self.pollRates.insert(ArdumowerROSSensors.ROS_SEN_BAT, 100)    # every 100ms

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
       elif mytype == '$LF:':
            self.timeLastROSCommand = rospy.get_time()
            self.lastFatalMessage = message[4:]
            rospy.logfatal(message[4:])

   # Methods process any incoming ROS response message (ex. Info messages)
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
       if items[2] == str(ArdumowerROSSensors.ROS_SEN_STATUS):
           print '[' + str(rospy.get_time()) + '] ' + message

       # Battery
       if items[2] == str(ArdumowerROSSensors.ROS_SEN_BAT):
           print '[' + str(rospy.get_time()) + '] ' + message


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

            # sleep for rate (Hz))
            sleeprate.sleep()
       self.close()


if __name__ == "__main__": 
    try:
        # Initialize Ardumower ROS Driver   
        robot = ArdumowerROSDriver(port="/dev/ttyACM0", baudrate=115200, timeout=0.5)

        # start observing serial port with given rate
        robot.spin(10)
    except KeyboardInterrupt:
        print "close connection to Ardumower"
        robot.close()