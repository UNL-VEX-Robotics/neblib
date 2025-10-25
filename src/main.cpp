/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       closm                                                     */
/*    Created:      5/21/2025, 1:14:46 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "neblib/xdrive.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;
controller controller1(primary);

vex::motor LFT = vex::motor(PORT12, ratio6_1, false);
vex::motor LFB = vex::motor(PORT13, ratio6_1, true);
vex::motor RFT = vex::motor(PORT3, ratio6_1, true);
vex::motor RFB = vex::motor(PORT2, ratio6_1, false);
vex::motor LBT = vex::motor(PORT14, ratio6_1, false);
vex::motor LBB = vex::motor(PORT15, ratio6_1, true);
vex::motor RBT = vex::motor(PORT5, ratio6_1, true);
vex::motor RBB = vex::motor(PORT4, ratio6_1, false);

vex::rotation parallel(PORT11);
vex::rotation perpendicular(PORT1);
vex::inertial imu(PORT16);
vex::distance dist(PORT17);

neblib::MCL mcl(
    { new neblib::Distance(dist, 0.0, 0.0, 0.0) },
    std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(parallel, 2.0)),
    2.0,
    std::unique_ptr<neblib::TrackerWheel>(new neblib::RotationTrackerWheel(perpendicular, 2.0)),
    2.0,
    imu,
    100,
    {
        neblib::Line(neblib::Point(-72.0, -72.0), neblib::Point(72.0, -72.0)),
        neblib::Line(neblib::Point(72.0, 72.0), neblib::Point(72.0, -72.0)),
        neblib::Line(neblib::Point(-72.0, 72.0), neblib::Point(72.0, 72.0)),
        neblib::Line(neblib::Point(-72.0, 72.0), neblib::Point(-72.0, -72.0))
    },
    1.0,
    0.1
);

neblib::XDrive xDrive(vex::motor_group(LFT, LFB), vex::motor_group(RFT, RFB), vex::motor_group(LBT, LBB), vex::motor_group(RBT, RBB), &mcl, imu);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {

  imu.calibrate();
  task::sleep(2000);

  mcl.setPose(-48.0, -24.0, 270);
  int time = Brain.Timer.time();
 
  while (true)
  {
    mcl.update();
    neblib::Pose e = mcl.getPose();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("x: ");
    Brain.Screen.print(e.x);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("y: ");
    Brain.Screen.print(e.y);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("h: ");
    Brain.Screen.print(e.heading);

    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("t: ");
    Brain.Screen.print(Brain.Timer.time() - time);
    time = Brain.Timer.time();
    
    task::sleep(10);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
