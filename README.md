# ArdumowerROS
ROS 2 based implementation of Ardumower DIY lawn mower


# Idea
After many years of running my Ardumower with Arduino (Ardumower Azurit) firmware, I wanted to try and learn something new. 
In common Ardumower firmware, all robot behaviors are part of Ardumower firmware. This runs pretty fine, but robot is some kind of stupid device 
(planned chaos). 
If robot hits an obstacle with bumpers, or if crossing perimeter wire, it stops, drives back and rotate randomly. This is true until battery is empty and robot returns to base.
This all works pretty good and involves much more code than it sounds like, however I want to drive the robot more intelligent by implementing
SLAM, path planning and maybe obstacle detection and collision avoidance with OpenCV.

Because all these tasks need lot of computing power which Arduino doesn't provide, a Raspberry PI gets used to run ROS (Robot Operating System)

## Benefits of new firmware
- as ROS node is implemented in Python, there will be no need to compile and flash Arduino over and over again during test. This is only true when new firmware is finished
- try different ROS packages for SLAM, mapping, navigation, path planning and so on

# Current state of development
Firmware has been prepared to fit ROS 2. Ardumower uses uart over hardware serial 2 to communicate with ROS 2. There are three ways of operating the system. You can run anything on a Raspberry PI (or other) which is part of the robot. You can operate some parts on a Raspberry PI and other ROS2 nodes on a local, stationary PC. Or you can use a ESP32 device running micro-ros to act as a bridge between your robot and a stationary ROS 2 system (PC or Raspberry PI). 
If running on a ESP32, it simply reads and writes to uart of Ardumower and publishes/subscribes to only two topics (ardumower_uart_tx and ardumower_uart_rx).

In any case, there is a ROS 2 node named "Ardumower driver", which process the uart data (either because it is attached directly or by subscribing to ardumower_uart_rx) and translate them to custom ROS 2 Ardumower messages. Some of these messages gets subscribed by a ROS 2 node called "base controller". Its task is currently translating some of the ardumower messages into common ROS 2 messages, like sensors_msgs and to do some tf transformations for odometry and IMU

![ROS2 schematics](https://github.com/PaddyCube/ArdumowerROS/blob/Development/Docs/ROS2_diagram.jpg)

# Content of repository
This repository contains (will contain) 
- Ardumower firmware
- ROS 2 packages
- URDF for simulation
- files and instructions to build an ArdumowerTurlebot as R&D platform

# Next steps
- Get ESP32 part done
- create new firmware (based on latest Azurit) by dropping all behaviors, simplify it to be ROS enabled
- create a ROS package for simple teleoperating scenarios
- add some safety new firmware, so robot stops if connection lost to ROS master, if perimeter was crossing, bumper hits obstacles and other 
  critical situations
