#!/usr/bin/env python

#Ardumower base controller
# This package contains a class to communicate with Ardumower driver
# It acts as a bridge between Ardumower driver and eco system as it 
# is a wrapper between Ardumower and ROS eco system for odometry and cmd_vel

# This code heavily bases on ros arduino bridge by hbrobotics
# https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_python/src/ros_arduino_python/base_controller.py

import roslib # roslib.load_manifest('ardumower_driver') <- what's this?
import rospy
import os
from ardumower_driver import ArdumowerROSDriver

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

# Ardumower
#from ardumower_ros import msg
from ardumower_ros import msg

""" Class to receive Twist commands and publish Odometry data """
class BaseController:
    def __init__(self, ardumower, base_frame, name="base_controllers"):

        self.ardumower = ardumower
        self.name = name
        self.base_frame = base_frame
        self.rate = float(rospy.get_param("~base_controller_rate", 10))
        self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
        self.stopped = False
                 
        pid_params = dict()
        pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", 0.067) 
        pid_params['wheel_track'] = rospy.get_param("~wheel_track", 0.23)
        pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", 11) 
        pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 35.0)
        pid_params['Kp'] = rospy.get_param("~Kp", 20)
        pid_params['Kd'] = rospy.get_param("~Kd", 12)
        pid_params['Ki'] = rospy.get_param("~Ki", 0)
        pid_params['Ko'] = rospy.get_param("~Ko", 50)
        
        self.accel_limit = rospy.get_param('~accel_limit', 0.1)
               
        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)
            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = rospy.Time.now()    
        self.then = now # time for determining dx/dy
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # Subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        rospy.Subscriber("ardumower_odometry", msg.odometry, self.ardumowerOdomCallBack)
        
        # Clear any old odometry info
        #self.arduino.reset_encoders()
        # get the even latest Odometry value from robot
        self.ardumower.pollSensor(ardumower.SEN_ODOM)
        
        # Set up the odometry broadcaster
        self.odomPub = rospy.Publisher('odom', Odometry, queue_size=5)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.encoder_resolution) + " ticks per rev")
        rospy.loginfo("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True
        
        if missing_params:
            os._exit(1)
                
        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']
        
        self.Kp = pid_params['Kp']
        self.Kd = pid_params['Kd']
        self.Ki = pid_params['Ki']
        self.Ko = pid_params['Ko']
        
       # self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            self.ardumower.pollSensor(ardumower.SEN_ODOM)
            # Read the encoders
            # try:
            #     left_enc, right_enc = self.arduino.get_encoder_counts()
            # except:
            #     self.bad_encoder_count += 1
            #     rospy.logerr("Encoder exception count: " + str(self.bad_encoder_count))
            #     return


    def ardumowerOdomCallBack(self, req):
        now = rospy.Time.now()
        # time since last call                
        dt = now - self.then
        self.then = now
        dt = dt.to_sec()
            
        # Calculate odometry
        if self.enc_left == None:
            dright = 0
            dleft = 0
        else:
            # distance per wheel
            dright = (req.rightTicks - self.enc_right) / self.ticks_per_meter
            dleft = (req.leftTicks - self.enc_left) / self.ticks_per_meter

        self.enc_right = req.rightTicks
        self.enc_left = req.leftTicks
            
        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt
                
        if (dxy_ave != 0):
            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave
            self.x += (cos(self.th) * dx - sin(self.th) * dy)
            self.y += (sin(self.th) * dx + cos(self.th) * dy)
    
        if (dth != 0):
            self.th += dth 
    
        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)
    
        # Create the odometry transform frame broadcaster.
        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0), 
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            self.base_frame,
            "odom"
            )
    
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.base_frame
        odom.header.stamp = now
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        self.odomPub.publish(odom)
            
            
    def stop(self):
        self.stopped = True
        self.ardumower.setMotors(0, 0, False)
            
    def cmdVelCallback(self, req):
        now = rospy.Time.now()
        # Handle velocity-based movement requests
        self.last_cmd_vel = rospy.Time.now()
        
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            
        #self.v_des_left = int(left * self.ticks_per_meter / self.ardumower.PID_RATE)
        #self.v_des_right = int(right * self.ticks_per_meter / self.ardumower.PID_RATE)
        self.v_des_left = int(left * self.ticks_per_meter )
        self.v_des_right = int(right * self.ticks_per_meter )


        if now > (self.last_cmd_vel + rospy.Duration(self.timeout)):

            self.v_des_left = 0
            self.v_des_right = 0
                
        if self.v_left < self.v_des_left:
            self.v_left += self.max_accel
            if self.v_left > self.v_des_left:
                self.v_left = self.v_des_left
        else:
            self.v_left -= self.max_accel
            if self.v_left < self.v_des_left:
                self.v_left = self.v_des_left
            
        if self.v_right < self.v_des_right:
            self.v_right += self.max_accel
            if self.v_right > self.v_des_right:
                self.v_right = self.v_des_right
        else:
            self.v_right -= self.max_accel
            if self.v_right < self.v_des_right:
                self.v_right = self.v_des_right
            
        # Set motor speeds in encoder ticks per PID loop
        if not self.stopped:
            print(self.v_left, ", " , self.v_right)
            self.ardumower.setMotors(self.v_left, self.v_right, False)
                
        self.t_next = now + self.t_delta

# only needed if direct started as basic test
    def spin(self, rate):
        rospy.loginfo("starting of sensors")
        r = rospy.Rate(self.rate)
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            self.ardumower.pollSerial()
            self.ardumower.pollSensor(self.ardumower.SEN_ODOM) 
            r.sleep()


if __name__ == "__main__": 
    try:
        # DEBUG ONLY
        rospy.init_node('ArdumowerTurtlebot', log_level=rospy.DEBUG)


        # Initialize Ardumower ROS Driver   
        driver = ArdumowerROSDriver(serialport="/dev/ttyACM0", baudrate=115200, timeout=0.5)
        # Make the connection
        driver.connect()
         # Initialize the base controller if used
        myBaseController = BaseController(driver, 'base_frame',  "_base_controller")

        # start observing serial port with given rate
        myBaseController.spin(100)
    except KeyboardInterrupt:
        print ("close connection to Ardumower")
        driver.close()     

                
        
        