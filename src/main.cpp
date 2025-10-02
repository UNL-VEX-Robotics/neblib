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

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

vex::inertial imu(PORT20);

neblib::TrackerWheel parallel(vex::rotation(PORT11), 2.0);
neblib::TrackerWheel perpendicular(vex::rotation(PORT10), 2.0);

neblib::Odometry odometry(&parallel, 1.625, &perpendicular, 0.75, &imu);

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

int task_a_odometry()
{
  while(true)
  {
    odometry.updatePose();
    task::sleep(10);
  }
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
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating... ");

  odometry.calibrate();
  odometry.setPose(0, 0, 0);

  vex::task a = vex::task(task_a_odometry);

  while (1) {
    neblib::Pose pose = odometry.getPose();

    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: ");
    Brain.Screen.print(pose.x);

    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Y: ");
    Brain.Screen.print(pose.y);

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("H: ");
    Brain.Screen.print(pose.heading);

    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("H2: ");
    Brain.Screen.print(imu.heading(deg));

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
