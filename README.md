# ArdumowerROS
ROS based implementation of Ardumower DIY lawn mower
This package can be used with ROS 1 (Noetic)

Hint: I stopped development for ROS 1 and continue using ROS 2

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
- Ardumower firmware
- ROS packages
- files and instructions to build an ArdumowerTurlebot as R&D platform

# Content of repository
This repository contains (will contain) 

- URDF for simulation


# Next steps
This package contains implementation for ROS 1 (Noetic).For the next steps, I must learn lots about ROS navigation. Because ROS 2 has already been released and thinks become more easy there, I stop development for ROS at this point.

Instead I'll bring the entire development to ROS 2 (Foxy). First of all, I migrate current state to ROS 2. Afterwards, I'll begin developing the more sophisticated topics.
