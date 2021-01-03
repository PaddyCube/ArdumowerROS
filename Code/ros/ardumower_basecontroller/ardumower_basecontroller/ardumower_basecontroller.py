#Ardumower base controller
# This package contains a class to communicate with Ardumower driver
# It acts as a bridge between Ardumower driver and eco system as it 
# is a wrapper between Ardumower and ROS eco system for odometry and cmd_vel

# This code heavily bases on ros arduino bridge by hbrobotics
# https://github.com/hbrobotics/ros_arduino_bridge/blob/indigo-devel/ros_arduino_python/src/ros_arduino_python/base_controller.py

import rclpy
from rclpy.node import Node
import os
from rclpy.duration import Duration
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_broadcaster import TransformStamped
from ardumower_driver import ardumower_driver

from math import sin, cos, pi
from geometry_msgs.msg import Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
import tf2_ros


# Ardumower

from ardumower_msgs import msg
from ardumower_msgs import srv

""" Class to receive Twist commands and publish Odometry data """
class BaseController(Node):
    def __init__(self, base_frame, name="ardumower_basecontroller"):

        super().__init__(name)
        self.declare_parameters(
            namespace='',
            parameters=[
                ("base_controller_rate", None),
                ("base_controller_timeout", None),
                ("wheel_diameter", None),
                ("wheel_track", None),
                ("encoder_resolution", None),
                ("gear_reduction", None),
                ("Kp", None),
                ("Kd", None),
                ("Ki", None),
                ("Ko", None),
                ("accel_limit", None)
            ])
        
        self.name = name
        self.base_frame = base_frame
        self.rate = self.get_parameter("base_controller_rate").get_parameter_value().integer_value
        self.timeout = self.get_parameter("base_controller_timeout").get_parameter_value().double_value
        self.stopped = False
                 
        
        self.wheel_diameter = self.get_parameter("wheel_diameter").get_parameter_value().double_value
        print(str(self.wheel_diameter))
        self.wheel_track = self.get_parameter("wheel_track").get_parameter_value().double_value
        self.encoder_resolution = self.get_parameter("encoder_resolution").get_parameter_value().integer_value
        self.gear_reduction = self.get_parameter("gear_reduction").get_parameter_value().integer_value
        self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
        self.Kd = self.get_parameter("Kd").get_parameter_value().double_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
        self.Ko = self.get_parameter("Ko").get_parameter_value().double_value
        
        self.accel_limit = self.get_parameter("accel_limit").get_parameter_value().double_value
               

            
        # How many encoder ticks are there per meter?
        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * pi)
        
        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
                
        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0
                        
        now = self.get_clock().now()
        self.then = now # time for determining dx/dy
        #self.t_delta = rospy.Duration(1.0 / self.rate)
        #self.t_next = now + self.t_delta

        # Internal data        
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0.0                      # position in xy plane
        self.y = 0.0
        self.th = 0.0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # Subscribers
        self.subOdom = self.create_subscription(msg.Odometry, "ardumower_odometry", self.ardumowerOdomCallBack,10) 
        self.subCmdVel = self.create_subscription(Twist, "cmd_vel", self.cmdVelCallback,10)
        
        # Clear any old odometry info
        #self.arduino.reset_encoders()
        # get the even latest Odometry value from robot
        #self.ardumower.pollSensor(ardumower.SEN_ODOM)
        
        # Set up the odometry broadcaster
        self.odomPub = self.create_publisher(Odometry, "odom", 10)
        self.odomBroadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Service clients
        self.motorClient = self.create_client(srv.SetMotor, "ardumower_driver/SetMotor")
        while not self.motorClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.motorRequest = srv.SetMotor.Request()
        
        
        self.get_logger().info("Started base controller for a base of " + str(self.wheel_track) + "m wide with " + str(self.gear_reduction * self.encoder_resolution) + " ticks per rev")
        self.get_logger().info("Publishing odometry data at: " + str(self.rate) + " Hz using " + str(self.base_frame) + " as base frame")
        
        timer_period = 1 / 100  # Interval timer for main loop
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def ardumowerOdomCallBack(self, req):
        now = self.get_clock().now()
        # time since last call                
        dt = now - self.then
        self.then = now
        #dt = dt.to_sec()
        dt = dt.nanoseconds / 1e9
        
            
        # Calculate odometry
        if self.enc_left == None:
            dright = 0
            dleft = 0
        else:
            # distance per wheel
            dright = (req.right_ticks - self.enc_right) / self.ticks_per_meter
            dleft = (req.left_ticks - self.enc_left) / self.ticks_per_meter

        self.enc_right = req.right_ticks
        self.enc_left = req.left_ticks
            
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
        transformStamped = TransformStamped()
        transformStamped.header.stamp = now.to_msg()
        transformStamped.header.frame_id = "odom"
        transformStamped.child_frame_id = "base_link"
        transformStamped.transform.translation.x = self.x
        transformStamped.transform.translation.y = self.y
        transformStamped.transform.translation.z = 0.0
        transformStamped.transform.rotation.x = quaternion.x
        transformStamped.transform.rotation.y = quaternion.y
        transformStamped.transform.rotation.z = quaternion.z
        transformStamped.transform.rotation.w = quaternion.w
        
        self.odomBroadcaster.sendTransform(transformStamped)
    
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.base_frame
        odom.header.stamp = now.to_msg()
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        self.odomPub.publish(odom)
            
            
    def stop(self):
        print("Stop")
        self.stopped = True
        self.motorRequest.left_pwm = 0
        self.motorRequest.right_pwm = 0
        self.motorRequest.mow_enable = False
        self.future = self.motorClient.call_async(self.motorRequest)
        
      
    def timer_callback(self):
        now = self.get_clock().now()
        if now > (self.last_cmd_vel + Duration(seconds=self.timeout, nanoseconds=0)):
            self.v_des_left = 0
            self.v_des_right = 0
            self.stop()
            self.last_cmd_vel = now
            
    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        self.last_cmd_vel = self.get_clock().now()
        now = self.last_cmd_vel
        
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
        print("send motor command")
        if not self.stopped:
            print(int(float(self.v_left/1090)*255), ", " , int(float(self.v_right/1090)*255))
            
            self.motorRequest.left_pwm = int(float(self.v_left/1090)*255)
            self.motorRequest.right_pwm = int(float(self.v_right/1090)*255)
            self.motorRequest.mow_enable = False
            self.future = self.motorClient.call_async(self.motorRequest)
        #self.t_next = now + self.t_delta


def main(args=None):
    rclpy.init(args=args)
    base_controller = BaseController("base_frame", name="ardumower_basecontroller" )

    rclpy.spin(base_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
