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
nothing developed so far, still learning ROS

# Content of repository
This repository contains (will contain) 
- Ardumower firmware
- ROS packages
- URDF for simulation
- files and instructions to build an ArdumowerTurlebot as R&D platform

# Next steps
- build a test platform (real robot) using Ardumower Hardware (PCB)
- create new firmware (based on latest Azurit) by dropping all behaviors, simplify it to be ROS enabled
- define needed ROS messages which should be transferred using rosserial
- create a ROS package for simple teleoperating scenarios
- add some safety new firmware, so robot stops if connection lost to ROS master, if perimeter was crossing, bumper hits obstacles and other 
  critical situations
