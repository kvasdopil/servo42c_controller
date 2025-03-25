# Servo42c robot arm controller

## Overview

A ROS2 controller for a robot arm, utilizing SERVO42C v1.0 controllers for stepper motors.

At the moment only works with v.1.0 of protocol (no crc)

# TODO
- add IK via moveit
  - setup moveit config
  - moveit should control joints
  - moveit should be controllable via foxglove
- add calibration via accelerometers
  - add reset command for servos
  - add a package to query from acc
- fix faulty servo
- test manual acceleration control
- try basic velocity control for servo v.1.0
- test servo42c v.1.1
  - test basic protocol
  - test speed control
  - test acceleration control
