#include "neblib/xdrive.hpp"

neblib::XDrive::XDrive(vex::motor_group &&leftFront, vex::motor_group &&rightFront, vex::motor_group &&leftBack, vex::motor_group &&rightBack, neblib::Odometry *odometry, vex::inertial *imu):
leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
{}

neblib::XDrive::XDrive(vex::motor &leftFront, vex::motor &rightFront, vex::motor &leftBack, vex::motor &rightBack, neblib::Odometry *odometry, vex::inertial *imu):
leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
{}