#include "neblib/xdrive.hpp"

neblib::XDrive::XDrive(vex::motor_group &&leftFront, vex::motor_group &&rightFront, vex::motor_group &&leftBack, vex::motor_group &&rightBack, neblib::Odometry *odometry, vex::inertial *imu) : leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
{
}

neblib::XDrive::XDrive(vex::motor &leftFront, vex::motor &rightFront, vex::motor &leftBack, vex::motor &rightBack, neblib::Odometry *odometry, vex::inertial *imu) : leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
{
}

void neblib::XDrive::driveLocal(float drive, float strafe, float turn, vex::velocityUnits unit)
{
    leftFront.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    rightFront.spin(vex::directionType::fwd, drive - drive - turn, unit);
    leftBack.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    rightBack.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::driveLocal(float drive, float strafe, float turn, vex::voltageUnits unit)
{
    leftFront.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    rightFront.spin(vex::directionType::fwd, drive - drive - turn, unit);
    leftBack.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    rightBack.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::drvieAngle(float velocity, float deg, float turn, vex::velocityUnits unit)
{
    float x = velocity * cosf(neblib::toRad(deg));
    float y = velocity * sinf(neblib::toRad(deg));
    this->driveLocal(y, x, turn, unit);
}

void neblib::XDrive::driveAngle(float velocity, float deg, float turn, vex::voltageUnits unit)
{
    float x = velocity * cosf(neblib::toRad(deg));
    float y = velocity * sinf(neblib::toRad(deg));
    this->driveLocal(y, x, turn, unit);
}

void neblib::XDrive::driveGlobal(float drive, float strafe, float turn, vex::velocityUnits unit)
{
    this->drvieAngle(hypotf(drive, strafe), -neblib::toRad(imu->heading(vex::rotationUnits::deg)), turn, unit);
}

void neblib::XDrive::driveGlobal(float drive, float strafe, float turn, vex::voltageUnits unit)
{
    this->driveAngle(hypotf(drive, strafe), -neblib::toRad(imu->heading(vex::rotationUnits::deg)), turn, unit);
}