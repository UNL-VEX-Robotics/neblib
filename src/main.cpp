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

motor leftFrontTop(PORT12, ratio6_1, false);
motor leftFrontBottom(PORT13, ratio6_1, true);
vex::motor_group leftFront(leftFrontTop, leftFrontBottom);
motor rightFrontTop(PORT3, ratio6_1, true);
motor rightFrontBottom(PORT2, ratio6_1, false);
vex::motor_group rightFront(rightFrontTop, rightFrontBottom);
motor leftBackTop(PORT14, ratio6_1, false);
motor leftBackBottom(PORT15, ratio6_1, true);
vex::motor_group leftBack(leftBackTop, leftBackBottom);
motor rightBackTop(PORT5, ratio6_1, true);
motor rightBackBottom(PORT4, ratio6_1, false);
vex::motor_group rightBack(rightBackTop, rightBackBottom);

inertial imu(PORT16);

neblib::TrackerWheel parallel(rotation(PORT11), 2.0);
neblib::TrackerWheel perpendicular(rotation(PORT1), 2.0);
neblib::Odometry odometry(&parallel, -3.5, &perpendicular, -1.1, &imu);

neblib::XDrive xDrive(leftFront, rightFront, leftBack, rightBack, &odometry, &imu);

neblib::PID rotationalPID(neblib::PID::Gains(0.135, 0.02, 0.25, 10.0), neblib::PID::SettleTimeExitConditions(0.25, 50, 10));
neblib::PID linearPID(neblib::PID::Gains(0.3, 0.001, 0.5, 12.0, true), neblib::PID::SettleTimeExitConditions(0.25, 30, 10));

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

int task_u_controllerPrint()
{
  while (true)
  {
    controller1.Screen.clearScreen();

    neblib::Pose pose = odometry.getPose();
    controller1.Screen.setCursor(1, 1);
    controller1.Screen.print("Position: ");
    controller1.Screen.setCursor(2, 1);
    controller1.Screen.print("(");
    controller1.Screen.print(pose.x);
    controller1.Screen.print(", ");
    controller1.Screen.print(pose.y);
    controller1.Screen.print(", ");
    controller1.Screen.print(pose.heading);
    controller1.Screen.print(")");

    controller1.Screen.setCursor(3, 1);
    controller1.Screen.print(parallel.getPosition() / neblib::toRad(imu.rotation() - 90));

    task::sleep(10);
  }
}

void usercontrol(void) {
  xDrive.setRotationalPID(&rotationalPID);
  xDrive.setLinearPID(&linearPID);
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print("Calibrating... ");
  
  
  odometry.calibrate(5);
  odometry.setPose(0, 0, 90);
  
  task a = task(task_a_odometry);
  
  // xDrive.driveToPose(24, 24, 90, 2);
  // xDrive.driveToPose(24, 72, 90, 2);
  // xDrive.driveToPose(0, 96, 90, 2);
  // xDrive.driveToPose(-24, 72, 90, 2);
  // xDrive.driveToPose(-24, 24, 90, 2);
  // xDrive.driveToPose(0, 0, 90, 2);
  

  while (true)
  {
    xDrive.driveGlobal(controller1.Axis3.position(percent) * 0.12, controller1.Axis4.position(percent) * 0.12, controller1.Axis1.position(percent) * 0.12, vex::voltageUnits::volt);
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
