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
#include "neblib/devices/tracker_wheel.hpp"
#include "neblib/auton_selector.hpp"
#include <iostream>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

vex::controller controller1(primary);

vex::motor frontLeftTop = vex::motor(PORT1, ratio6_1, false);
vex::motor frontLeftBottom = vex::motor(PORT2, ratio6_1, true);
vex::motor frontRightTop = vex::motor(PORT4, ratio6_1, true);
vex::motor frontRightBottom = vex::motor(PORT3, ratio6_1, false);
vex::motor backLeftTop = vex::motor(PORT18, ratio6_1, false);
vex::motor backLeftBottom = vex::motor(PORT17, ratio6_1, true);
vex::motor backRightTop = vex::motor(PORT13, ratio6_1, true);
vex::motor backRightBottom = vex::motor(PORT12, ratio6_1, false);

vex::motor_group leftFront(frontLeftTop, frontLeftBottom);
vex::motor_group rightFront(frontRightTop, frontRightBottom);
vex::motor_group leftBack(backLeftTop, backLeftBottom);
vex::motor_group rightBack(backRightTop, backRightBottom);

vex::inertial imu(PORT10, vex::turnType::right);
vex::rotation parallelRotation(PORT6, true); // 6
vex::rotation perpendicularRotation(PORT8, true); //8

neblib::RotationTrackerWheel parallel(
    parallelRotation,
    2.05);
neblib::RotationTrackerWheel perpendicular(
    perpendicularRotation,
    2.0119);

neblib::Odometry odom(
    parallel,
    -4.0, // -4.0
    perpendicular,
    0.0, //0.0
    imu);
neblib::XDrive xDrive(
    leftFront,
    rightFront,
    leftBack,
    rightBack,
    imu,
    &odom);

neblib::PID linearPID(
    neblib::PID::Gains(
        0.4,
        0.005,
        0.8,
        0.45),
    neblib::PID::Behaviors(
        12.0,
        true),
    neblib::PID::ExitConditions(
        0.25,
        30));

neblib::PID angularPID(
    neblib::PID::Gains(
        0.15,
        0.005,
        0.2),
    neblib::PID::Behaviors(
        15.0,
        true),
    neblib::PID::ExitConditions(
        0.5,
        50));
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void)
{
    xDrive.setLinearController(&linearPID);
    xDrive.setAngularController(&angularPID);
    // neblib::Page redPage = neblib::Page(neblib::Button(0, 0, 160, 50, vex::color(155, 155, 155), vex::color(50, 50, 50), vex::color(255, 255, 255), vex::color(0, 0, 0), "Red"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red AWP"),
    //                                                                                                                                                                               neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Red Elim")});

    // neblib::Page bluePage = neblib::Page(neblib::Button(160, 0, 160, 50, vex::color(155, 155, 155), vex::color(50, 50, 50), vex::color(255, 255, 255), vex::color(0, 0, 0), "Blue"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue AWP"),
    //                                                                                                                                                                                   neblib::Button(310, 120, 160, 50, vex::color(0, 0, 0), vex::color(0, 0, 150), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Blue Elim")});

    // neblib::Page skillsPage = neblib::Page(neblib::Button(320, 0, 160, 50, vex::color(155, 155, 155), vex::color(50, 50, 50), vex::color(255, 255, 255), vex::color(0, 0, 0), "Skills"), {neblib::Button(10, 120, 160, 50, vex::color(0, 0, 0), vex::color(150, 0, 0), vex::color(255, 255, 255), vex::color(255, 255, 255), "Left Skills")});
    // neblib::AutonSelector selector = neblib::AutonSelector(Brain,
    //                                                        {&redPage, &bluePage, &skillsPage},
    //                                                        neblib::Button(180, 120, 120, 50, vex::color(255, 255, 255), vex::color(0, 0, 0), vex::color(0, 0, 0), vex::color(255, 255, 255), "Calibrate"));

    // selector.runSelector();
    // Brain.Screen.clearScreen();
    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print("Auton: ");
    // Brain.Screen.print(selector.getAuton());
    // Brain.Screen.setCursor(2, 1);
    // Brain.Screen.print("Color: ");
    // Brain.Screen.print(selector.getColor() == vex::color::blue ? "Blue" : "Red");
    // // All activities that occur before the competition starts
    // // Example: clearing encoders, setting servo positions, ...
}

void autonomous(void)
{
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

void usercontrol(void)
{
    odom.calibrate();
    odom.setPose(
        0.0,
        0.0,
        90.0);
    task a = neblib::launchTask(std::bind(&neblib::Odometry::begin, &odom));
    int out = xDrive.driveToPose(23.5, 0.0, 90.0, 2500);
    controller1.Screen.print(out);
    xDrive.turnTo(90.0);

    task::sleep(1000);
    xDrive.stop(coast);

    while (true)
    {
        // xDrive.driveGlobal(
        //     controller1.Axis3.position(percent),
        //     controller1.Axis4.position(percent),
        //     controller1.Axis1.position(percent),
        //     vex::velocityUnits::pct);

        const neblib::Pose p = odom.getPose();
        Brain.Screen.clearScreen();
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("X: ");
        Brain.Screen.print(p.x);
        Brain.Screen.setCursor(2, 1);
        Brain.Screen.print("Y: ");
        Brain.Screen.print(p.y);
        Brain.Screen.setCursor(3, 1);
        Brain.Screen.print("T: ");
        Brain.Screen.print(p.heading);
        task::sleep(10);
    }
}

//
// Main will set up the competition functions and callbacks.
//
int main()
{
    // Set up callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Prevent main from exiting with an infinite loop.
    while (true)
    {
        wait(100, msec);
    }
}
