# ArdumowerROS Driver
# This package contains a class to communicate with Ardumower Hardware
# Package is used to communicate using Serial console and doesn't use any ROS packages at this point

from math import pi
from sys import argv
from rclpy.duration import Duration
import serial
import time

import rclpy
from rclpy.node import Node

from datetime import datetime
from datetime import timedelta
from ardumower_msgs import msg
from ardumower_msgs import srv
from std_msgs.msg import String

DEBUG = True


class ArdumowerROSDriver(Node):
    
   # connection types
   CONNECT_SERIAL, CONNECT_UART, CONNECT_MSG = range(0,3)
   
   # all types of ROS sensor messages
   SEN_STATUS, SEN_PERIM_LEFT, SEN_PERIM_RIGHT, SEN_LAWN_FRONT, SEN_LAWN_BACK, \
   SEN_BAT_VOLTAGE, SEN_CHG_CURRENT, SEN_CHG_VOLTAGE, \
   SEN_MOTOR_LEFT, SEN_MOTOR_RIGHT, SEN_MOTOR_MOW, \
   SEN_BUMPER_LEFT, SEN_BUMPER_RIGHT, SEN_DROP_LEFT, SEN_DROP_RIGHT, \
   SEN_SONAR_CENTER, SEN_SONAR_LEFT, SEN_SONAR_RIGHT, \
   SEN_BUTTON, SEN_IMU, SEN_ODOM, SEN_MOTOR_MOW_RPM, SEN_RTC, SEN_RAIN, SEN_TILT, SEN_FREE_WHEEL \
    = range(0, 26)

   # Error types
   ERR_MOTOR_LEFT, ERR_MOTOR_RIGHT, ERR_MOTOR_MOW, ERR_MOW_SENSE, \
   ERR_IMU_COMM, ERR_IMU_TILT, ERR_RTC_COMM, ERR_RTC_DATA, ERR_PERIMETER_TIMEOUT, \
   ERR_ODOMETRY_LEFT, ERR_ODOMETRY_RIGHT, ERR_BATTERY, ERR_CHARGER, ERR_GPS_COMM, \
   ERR_GPS_DATA, ERR_ADC_CALIB, ERR_IMU_CALIB, ERR_EEPROM_DATA, ERR_CPU_SPEED, ERR_ENUM_COUNT, \
   ERR_ROS = range(0, 21)

   # Ardumower States
   STATE_OFF, STATE_ROS, STATE_REMOTE, STATE_ERROR, STATE_STATION_CHARGING, STATE_STATION = range(
       0, 6)

   # Ardumower Event Type
   ROS_EV_NEW_STATE, ROS_EV_SENSOR_TRIGGER, ROS_EV_ERROR = range(0, 3)

   def __init__(self):

        super().__init__("ardumower_driver")
        self.declare_parameters(
            namespace='',
            parameters=[
                ("port", None),
                ("timeout", None),
                ("baudrate", None),
                ("connection_type", None),
                ("max_velocity", None),
                ("wheel_diameter", None),
                ("encoder_resolution", None),
                ("gear_reduction", None),
            ])
        
        self.connection_type = self.get_parameter("connection_type").get_parameter_value().integer_value
        print(self.connection_type)
        if self.connection_type == self.CONNECT_SERIAL:
            
            self.port = self.get_parameter("port").get_parameter_value().string_value
            print(self.port)
            self.baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value
            
        
        self.timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.encoder_resolution = self.get_parameter("encoder_resolution").get_parameter_value().integer_value
        self.gear_reduction = self.get_parameter("gear_reduction").get_parameter_value().integer_value
        self.wheel_diameter = self.get_parameter("wheel_diameter").get_parameter_value().double_value
        # calculate highest possible tick rate of motors
        # this gets used to translate velocity commands to PWM rate (255)
        self.max_ticks = self.max_velocity / (self.wheel_diameter + pi) * self.encoder_resolution * self.gear_reduction
        self.ArdumowerStatus = -1
        self.lastSensorTriggered = -1
        self.lastError = -1
        self.serialMessageErrorCounter = 0

        # define publishers here
        self.pubStatus = self.create_publisher(
            msg.Status, "ardumower_status", 10)
        self.pubBattery = self.create_publisher(
            msg.Battery, "ardumower_battery", 10)
        self.pubBumper = self.create_publisher(
            msg.Bumper, "ardumower_bumper", 10)
        self.pubPerimeter = self.create_publisher(
            msg.Perimeters, "ardumower_perimeters", 100)
        self.pubMotor = self.create_publisher(msg.Motor, "ardumower_motor", 10)
        self.pubSonar = self.create_publisher(msg.Sonar, "ardumower_sonar", 10)
        self.pubOdometry = self.create_publisher(
            msg.Odometry, "ardumower_odometry", 100)
        self.pubIMU = self.create_publisher(msg.Imu, "ardumower_imu", 10)

        #subscribers
        self.subArdumowerUart = self.create_subscription(String, "/ardumower_uart", self.UartCallback,10)
        
        # service definitions
        self.setMotorSrv = self.create_service(srv.SetMotor, "ardumower_driver/SetMotor", self.MotorCallback)
        # define mow motor status here
        self.mowMotorEnable = False

        self.connect()

        # ROS Info Messages
        self.ROSMessageID = 0
        self.lastReceivedMessageID = 0

        # ROS Timeouts
        self.timeoutROSMessage = 5  # await at least one message every x sec.
        # when last ros command has arrived
        self.timeLastROSCommand = self.get_clock().now()

        timer_period = 1 / 100  # Interval timer for main loop
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # Method to connect to serial console of Arduino
   def connect(self):
        if self.connection_type != self.CONNECT_SERIAL:
            return
        
        print("Connecting to Arduino on port", self.port,
              " with baud rate ", self.baudrate, "...")
        self.port = serial.Serial(
            port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.timeout)
        # The next line is necessary to give the firmware time to wake up.
        time.sleep(1)
        self.port.flush()
        time.sleep(5)

   # read all incoming messages on serial console
   # analyze kind of message and call corresponding method for further processing
   # check if any message needs to be send to Arduino and trigger command send
   def pollSerial(self):
       if self.connection_type != self.CONNECT_SERIAL:
           return
       
       while self.port.inWaiting() > 0:
          line = self.port.readline().decode('utf-8')
          if DEBUG:
              print(line)
          # Check for incoming ROS messages
          if line.startswith('$'):
              # check for command type
               if line.startswith('$LD') or line.startswith('$LI') or line.startswith('$LW') or line.startswith('$LE') or line.startswith('$LF'):
                 self.processInfoMessage(line)
               elif line.startswith('$RS'):
                   self.processResponseMessage(line)
               elif line.startswith('$EV'):
                   self.processEventMessage(line)

   def UartCallback(self, req):
       if self.connection_type != self.CONNECT_MSG:
           return
       if DEBUG:
           print(req.data)
          # Check for incoming ROS messages
       if req.data.startswith('$'):
           # check for command type
           if req.data.startswith('$LD') or req.data.startswith('$LI') or req.data.startswith('$LW') or req.data.startswith('$LE') or req.data.startswith('$LF'):
               self.processInfoMessage(req.data)
           elif req.data.startswith('$RS'):
               self.processResponseMessage(req.data)
           elif req.data.startswith('$EV'):
               self.processEventMessage(req.data)

   # Method process Info messages from Arduino into corresponding
   # ROS Log message

   def processInfoMessage(self, message):

       mtype = message[0:4]
       if mtype == '$LD:':
           self.timeLastROSCommand = self.get_clock().now()
           self.lastDebugMessage = message[4:]
           self.get_logger().debug(message[4:])
       elif mtype == '$LI:':
            self.timeLastROSCommand = self.get_clock().now()
            self.lastInfoMessage = message[4:]
            self.get_logger().info(message[4:])
       elif mtype == '$LW:':
            self.timeLastROSCommand = self.get_clock().now()
            self.lastWarnMessage = message[4:]
            self.get_logger().warn(message[4:])
       elif mtype == '$LE:':
            self.timeLastROSCommand = self.get_clock().now()
            self.lastErrorMessage = message[4:]
            self.get_logger().error(message[4:])
       elif mtype == '$LF:':
            self.timeLastROSCommand = self.get_clock().now()
            self.lastFatalMessage = message[4:]
            self.get_logger().fatal(message[4:])

   # Methods process any incoming ROS response message (w.o. Info messages)
   # First determine which type has been received. Then call related method for further processing
   def processResponseMessage(self, message):
       self.timeLastROSCommand = self.get_clock().now()
       items = message.split("|")
       # get message type
       # check message ID
       try:
           self.lastReceivedMessageID = items[1]
       
           if DEBUG:
               print(message)

           # Status message
           if items[2] == str(ArdumowerROSDriver.SEN_STATUS):
               msgStatus = msg.Status()
               msgStatus.header.stamp = self.get_clock().now().to_msg()
               msgStatus.loop_per_sec = int(items[3])
               msgStatus.state_id = int(items[4])
               msgStatus.state = items[5]
               self.pubStatus.publish(msgStatus)

           # Battery
           if (items[2] == str(ArdumowerROSDriver.SEN_BAT_VOLTAGE) or
               items[2] == str(ArdumowerROSDriver.SEN_CHG_CURRENT) or
               items[2] == str(ArdumowerROSDriver.SEN_CHG_VOLTAGE)):
               msgBattery = msg.Battery()
               msgBattery.header.stamp = self.get_clock().now().to_msg()
               msgBattery.voltage = float(items[3])
               msgBattery.charge_voltage = float(items[4])
               msgBattery.charge_current = float(items[5])
               self.pubBattery.publish(msgBattery)

           # Bumper
           if (items[2] == str(ArdumowerROSDriver.SEN_BUMPER_LEFT) or
               items[2] == str(ArdumowerROSDriver.SEN_BUMPER_RIGHT)):
               msgBumper = msg.Bumper()
               msgBumper.header.stamp = self.get_clock().now().to_msg()
               msgBumper.bumper_left_count = int(items[3])
               msgBumper.bumper_right_count = int(items[4])
               msgBumper.left_pressed = bool(items[5])
               msgBumper.right_pressed = bool(items[6])
               self.pubBumper.publish(msgBumper)

           # Perimeter
           if (items[2] == str(ArdumowerROSDriver.SEN_PERIM_LEFT) or
               items[2] == str(ArdumowerROSDriver.SEN_PERIM_RIGHT)):
               msgPeriLeft = msg.Perimeter()
               msgPeriRight = msg.Perimeter()
               msgPeri = msg.Perimeters()
               msgPeri.header.stamp = self.get_clock().now().to_msg()
               msgPeriLeft.inside = items[3] in ('1') # convert string to boolean
               msgPeriRight.inside = items[4] in ('1')
               msgPeriLeft.magnitude = int(items[5])
               msgPeriRight.magnitude = int(items[6])
               msgPeriLeft.signal_timeout = items[8] in ('1')
               msgPeriRight.signal_timeout = items[9] in ('1')

               msgPeri.data.append(msgPeriLeft)
               msgPeri.data.append(msgPeriRight)
               self.pubPerimeter.publish(msgPeri)

           # Motor
           if (items[2] == str(ArdumowerROSDriver.SEN_MOTOR_LEFT) or
               items[2] == str(ArdumowerROSDriver.SEN_MOTOR_RIGHT) or
               items[2] == str(ArdumowerROSDriver.SEN_MOTOR_MOW) or
               items[2] == str(ArdumowerROSDriver.SEN_MOTOR_MOW_RPM)):

               msgMotor = msg.Motor()
               msgMotor.header.stamp = self.get_clock().now().to_msg()
               msgMotor.left_pwm = int(float(items[3]))
               msgMotor.right_pwm = int(float(items[4]))
               msgMotor.motor_left_current = float(items[7])
               msgMotor.motor_right_current = float(items[8])
               msgMotor.motor_left_sense = float(items[5])
               msgMotor.motor_right_sense = float(items[6])

               msgMotor.overload_left = int(items[9])
               msgMotor.overload_right = int(items[10])

               msgMotor.mow_enable = bool(items[11])
               msgMotor.mow_current = float(items[12])
               msgMotor.mow_sense = float(items[13])
               msgMotor.overload_mow = int(items[14])
               self.pubMotor.publish(msgMotor)
           # Sonar
           if (items[2] == str(ArdumowerROSDriver.SEN_SONAR_CENTER) or
               items[2] == str(ArdumowerROSDriver.SEN_SONAR_LEFT) or
               items[2] == str(ArdumowerROSDriver.SEN_SONAR_RIGHT)):
               msgSonar = msg.Sonar()
               msgSonar.header.stamp = self.get_clock().now().to_msg()
               msgSonar.distance_left = int(items[3])
               msgSonar.distance_center = int(items[4])
               msgSonar.distance_right = int(items[5])
               self.pubSonar.publish(msgSonar)

           # Odometry
           if items[2] == str(ArdumowerROSDriver.SEN_ODOM):
               msgOdom = msg.Odometry()
               msgOdom.header.stamp = self.get_clock().now().to_msg()
               msgOdom.left_ticks = int(items[3])
               msgOdom.right_ticks = int(items[4])
               self.pubOdometry.publish(msgOdom)

           # IMU
           if items[2] == str(ArdumowerROSDriver.SEN_IMU):
               msgImu = msg.Imu()
               msgImu.header.stamp = self.get_clock().now().to_msg()
               msgImu.yaw = float(items[3])
               msgImu.pitch = float(items[4])
               msgImu.roll = float(items[5])
               msgImu.gyro_x = float(items[6])
               msgImu.gyro_y = float(items[7])
               msgImu.gyro_z = float(items[8])
               msgImu.acc_x = float(items[9])
               msgImu.acc_y = float(items[10])
               msgImu.acc_z = float(items[11])
               msgImu.compass_x = float(items[12])
               msgImu.compass_y = float(items[13])
               msgImu.compass_z = float(items[14])
               self.pubIMU.publish(msgImu)

       except IndexError:
            self.serialMessageErrorCounter+=1
            self.get_logger().error("Message structure invalid " + message )
            self.get_logger().error("No of invalid messages received: " + str(self.serialMessageErrorCounter) )
       except ValueError:
            self.serialMessageErrorCounter+=1
            self.get_logger().error("Message structure invalid " + message )
            self.get_logger().error("No of invalid messages received: " + str(self.serialMessageErrorCounter) )
            


   # Method process any incoming Event message which has been raised by Ardumower
   def processEventMessage(self, event):

       if DEBUG:
           
           print (event)
       self.timeLastROSCommand = self.get_clock().now()
       items = event.split("|")

       # determine type of event
       if items[1] == str(ArdumowerROSDriver.ROS_EV_NEW_STATE):
           self.ArdumowerStatus = int(items[2])
       
       if items[1] == str(ArdumowerROSDriver.ROS_EV_SENSOR_TRIGGER):
           self.lastSensorTriggered = int(items[2])
           self.pollSensor(self.lastSensorTriggered)
           
    
       if items[1] == str(ArdumowerROSDriver.ROS_EV_ERROR):
           self.lastError = int(items[2])


   # Method to set motors
   def setMotors(self, leftPWM,rightPWM, enableMowMotor):
       self.ROSMessageID+=1
       cmd = ( '$M1|' + str(self.ROSMessageID) + '|' + str(leftPWM) + '|' + str(rightPWM) + 
            '|' + str(enableMowMotor) + '\r\n' )
       self.port.write(cmd.encode())   

   # Method to poll a sensor
   # Create command for request string and send it by serial console to Arduino
   def pollSensor(self, sensorID):
       if self.connection_type != self.CONNECT_SERIAL:
           return
       
       self.ROSMessageID+=1
       cmd = '$RQ|' + str(self.ROSMessageID) + '|' + str(sensorID)+ '\r\n'
       self.port.write(cmd.encode('utf-8'))

   def close(self):
        if self.connection_type != self.CONNECT_SERIAL:
            return
        print("disconnect from serial port")
        self.port.close()

   def MotorCallback(self, req, resp):
       leftpwm = int(float(req.left_pwm / self.max_ticks) * 255)
       rightpwm = int (float(req.right_pwm / self.max_ticks) * 255 )
       # check for max possible pwm value 
       if leftpwm < -255 or leftpwm > 255:
           # if left pwm is out of range, adjust rightpwm accordingly
           factor = abs(float(255 / leftpwm))
           rightpwm = int(rightpwm * factor )
           if leftpwm < 0:
               leftpwm = -255
           else:    
               leftpwm = 255
           
       if rightpwm < -255 or rightpwm > 255:
           # if rightpwm is out of range, adjust leftpwm accordingly
           factor = abs(float(255 / rightpwm))
           leftpwm = int(leftpwm * factor )
           if rightpwm < 0:
               rightpwm = -255
           else:    
               rightpwm = 255
       
       if DEBUG:
           print(leftpwm, ", ", rightpwm)
       self.setMotors(leftpwm, rightpwm, req.mow_enable)
       return resp

# only needed if direct started as basic test
   def timer_callback(self):

       self.pollSerial()
       # check for timeout
       if self.get_clock().now() - self.timeLastROSCommand  > Duration(seconds= self.timeoutROSMessage, nanoseconds= 0):
           self.get_logger().fatal("Message timeout, no messages from Ardumower received")
   

def main(args=None):
    rclpy.init(args=args)
    ardumower_driver = ArdumowerROSDriver()

    rclpy.spin(ardumower_driver)
    ardumower_driver.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
