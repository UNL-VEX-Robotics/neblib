# neblib
A VEX V5 library created by VURC team SKERS. The intent of this library is to minimize the amount of code that needs rewritten between robots

## Features
* PID class with multiple exit condition types
* Odometry class to track the position of a robot
  * Tracker Wheel class to wrap both vex::rotation and vex::encoder
* X-Drive class with basic autonomous movements and user inputs

## Requirements for Use
This library is designed specifically for use within VEX Robotics teams who fulfill at least one of the following:

* Understand to a level where they can explain the different algorithms and control methods used in this library.  
  * This is to ensure that teams are following the RECF student centered policy.
* Contribute to the library.
  * To contribute to the library, please contact skers.vurc@gmail.com.

## Developement
If you are developing neblib, please use seperate branches, do not commit/push to main unless your branch ***is fully functional***. Please use the following branch name conventions:  

`<name, GitHub username, or team>-<type>-<area of work>`

These are the different types:
* developement - changing the functionality of a class
* feature - adding on to a class or creating new ones
* bugfix - fixing unintentional behavior within classes

An example of a branch name:   
`bclosman-bugfix-odometry`  
This would be read as 'bclosman is working on a bugfix with the odometry'.


## Notes and Warnings
The `main.cpp` file is used during prototyping. 
Code not normally found within the VEX Competition Template can be deleted or written over with no consequence.s