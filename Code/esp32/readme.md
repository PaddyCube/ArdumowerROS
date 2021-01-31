This package contains firmware for an ESP32 device which runs micro-ROS to bridge Arduino to ROS2 ecosystem.

Arduino Firmware of ArdumowerTurleBot uses Serial communication to send sensor data and to receive commands. This commands needs to be transferred to ROS2. 
As one option, you can use ESP32 device running micro-ROS. Simply connect ESP32 UART to Ardumower PCB (1.2 or later) Wifi connection port (P44 at PCB 1.2)

ESP32 will receive all Messages from Arduino via UART and translate them into corresponsing ROS2 Messages (custom Ardumower messages) and vice-a-verse.
Main ROS2 components will run outside the robot, for example on a PC or Raspberry PI in your office.


To compile this, you need ESP IDF as well as micro-ROS for ESP-IDF found here
https://github.com/micro-ROS/micro_ros_espidf_component

Simply clone this git repo into this ESP32 package and use 
idf.py menuconfig
idf.py build
idf.py flash
