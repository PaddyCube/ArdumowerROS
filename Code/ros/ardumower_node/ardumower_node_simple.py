#!/usr/bin/env python

#Ardumower node simple
# This is a simple Ardumower node. It will not provide much, only
# read sensor values by a defined interval
# also it will do basic behavior like stop on bumper etc.

# This code heavily bases on ros arduino bridge by hbrobotics
# https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_python


import rospy

from ardumower_driver.ardumower_driver import ArdumowerROSDriver
from ardumower_driver.ardumower_Base_controller import BaseController
from geometry_msgs.msg import Twist
import os, time
#import thread
from serial.serialutil import SerialException

from ardumower_ros import msg

class ArdumowerROS():
    def __init__(self):
        rospy.init_node('ArdumowerTurtlebot', log_level=rospy.DEBUG)

        # Get the actual node name in case it is set in the launch file
        self.name = rospy.get_name()

        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)

        self.port = rospy.get_param("~port", "/dev/ttyU")
        self.baud = int(rospy.get_param("~baud", 115200))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')
        
        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Initialize the controlller
        self.driver = ArdumowerROSDriver(self.port, self.baud, self.timeout)

        # Make the connection
        self.driver.connect()

        # Reserve a thread lock
        # mutex = thread.allocate_lock() ? whats this good for

        # ROS poll rates in Hz
        # add any sensor you want to poll to parameter file
        # Initialize any sensors
        self.mySensors = list()
        sensor_params = rospy.get_param("~sensors", dict({}))
        self.pollRates = [0] * 26

        for name, params in sensor_params.items():
            # Set the direction to input if not specified
            # try:
            #     params['SensorID']
            # except:
            #     params['direction'] = 'input'
            self.pollRates.insert(params['SensorID'], params['rate'])
            
        # time of last poll
        # this list holds the time for the next sensor poll, based on
        # list with ROS poll rates
        self.timeNextPoll = []
        for index, times in enumerate(self.pollRates):
            self.timeNextPoll.insert(index, rospy.get_time() + 10 )

        # Initialize the base controller if used
        self.myBaseController = BaseController(self.driver, self.base_frame, self.name + "_base_controller")

    def spin(self):
        rospy.loginfo("starting of sensors")
        r = rospy.Rate(self.rate)
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():

            currenttime = rospy.get_time()
            # poll for sensors
            # loop through list with poll rates. If poll rate != 0 and
            # corresponding timeout in list timeNextPoll has been exceeded, 
            # request new data for the actual sensor
            for index, polls in enumerate(self.pollRates):
                if currenttime > self.timeNextPoll[index] and polls != 0:
                    self.timeNextPoll[index] = currenttime + 1/float(polls)
                    self.driver.pollSensor(index) 

            r.sleep()

   
    def shutdown(self):
        rospy.loginfo("Shutting down Arduino Node...")

        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass

        # Close the serial port
        try:
            self.driver.close()
        except:
            pass
        finally:
            rospy.loginfo("Serial port closed.")
            os._exit(0)

if __name__ == '__main__':
    try:
        myArdumower = ArdumowerROS()
        myArdumower.spin()
    except SerialException:
        rospy.logerr("Serial exception trying to open port.")
        os._exit(0)
