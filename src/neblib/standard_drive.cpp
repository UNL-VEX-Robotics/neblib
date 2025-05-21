#include "neblib/standard_drive.hpp"

neblib::StandardDrive::StandardDrive(vex::motor_group &&leftMotorGroup, vex::motor_group &&rightMotorGroup, neblib::IMUWrapper &&IMU) : leftMotorGroup(leftMotorGroup), rightMotorGroup(rightMotorGroup), IMU(IMU) {} 

void neblib::StandardDrive::driveTank(double leftInput, double rightInput, vex::voltageUnits unit)
{
    leftMotorGroup.spin(vex::directionType::fwd, leftInput, unit);
    rightMotorGroup.spin(vex::directionType::fwd, rightInput, unit);
}

void neblib::StandardDrive::driveTank(double leftInput, double rightInput, vex::velocityUnits unit)
{
    leftMotorGroup.spin(vex::directionType::fwd, leftInput, unit);
    rightMotorGroup.spin(vex::directionType::fwd, rightInput, unit);
}

void neblib::StandardDrive::driveArcade(double driveInput, double turnInput, vex::voltageUnits unit)
{
    leftMotorGroup.spin(vex::directionType::fwd, driveInput + turnInput, unit);
    rightMotorGroup.spin(vex::directionType::fwd, driveInput - turnInput, unit);
}

void neblib::StandardDrive::driveArcade(double driveInput, double turnInput, vex::velocityUnits unit)
{
    leftMotorGroup.spin(vex::directionType::fwd, driveInput + turnInput, unit);
    rightMotorGroup.spin(vex::directionType::fwd, driveInput - turnInput, unit);
}