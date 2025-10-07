#include "neblib/xdrive.hpp"
#include <iostream>

neblib::XDrive::XDrive(vex::motor_group leftFront, vex::motor_group rightFront, vex::motor_group leftBack, vex::motor_group rightBack, neblib::Odometry *odometry, vex::inertial *imu) : leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
{
}

neblib::XDrive::XDrive(vex::motor &leftFront, vex::motor &rightFront, vex::motor &leftBack, vex::motor &rightBack, neblib::Odometry *odometry, vex::inertial *imu) : leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
{
}

void neblib::XDrive::setLinearPID(neblib::PID *pid)
{
    linearPID = pid;
}

void neblib::XDrive::setRotationalPID(neblib::PID *pid)
{
    rotationalPID = pid;
}

void neblib::XDrive::driveLocal(float drive, float strafe, float turn, vex::velocityUnits unit)
{
    leftFront.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    rightFront.spin(vex::directionType::fwd, drive - strafe - turn, unit);
    leftBack.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    rightBack.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::driveLocal(float drive, float strafe, float turn, vex::voltageUnits unit)
{
    leftFront.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    rightFront.spin(vex::directionType::fwd, drive - strafe - turn, unit);
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

void neblib::XDrive::driveGlobal(float x, float y, float turn, vex::velocityUnits unit)
{
    this->drvieAngle(hypotf(x, y), neblib::toDeg(atan2f(x, y)) + imu->heading(vex::rotationUnits::deg) - 90, turn, unit);
}

void neblib::XDrive::driveGlobal(float x, float y, float turn, vex::voltageUnits unit)
{
    this->driveAngle(hypotf(x, y), neblib::toDeg(atan2f(x, y)) + imu->heading(vex::rotationUnits::deg) - 90, turn, unit);
}

void neblib::XDrive::stop(vex::brakeType stopType)
{
    leftFront.stop(stopType);
    rightFront.stop(stopType);
    leftBack.stop(stopType);
    rightBack.stop(stopType);
}

float neblib::XDrive::turnFor(float deg, float minOutput, float maxOutput, float timeout)
{
    rotationalPID->reset();
    float target = imu->rotation(vex::rotationUnits::deg) + deg;
    float time = 0.0;

    while (!rotationalPID->isSettled() && time < timeout)
    {
        float error = target - imu->rotation(vex::rotationUnits::deg);
        float output = rotationalPID->getOutput(error, minOutput, maxOutput);

        this->driveLocal(0.0, 0.0, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

float neblib::XDrive::turnFor(float deg, float timeout)
{
    return this->turnFor(deg, -infinityf(), infinityf(), timeout);
}

float neblib::XDrive::turnTo(float heading, float minOutput, float maxOutput, float timeout)
{
    rotationalPID->reset();
    float time = 0.0;

    while (!rotationalPID->isSettled() && time < timeout)
    {
        float error = neblib::wrap(heading - imu->heading(vex::rotationUnits::deg), -180, 180);
        float output = rotationalPID->getOutput(error, minOutput, maxOutput);

        this->driveLocal(0.0, 0.0, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

float neblib::XDrive::turnTo(float heading, float timeout)
{
    return this->turnTo(heading, -infinityf(), infinityf(), timeout);
}

float neblib::XDrive::driveTo(float x, float y, float minOutput, float maxOutput, float timeout)
{
    return this->driveToPose(x, y, imu->heading(vex::rotationUnits::deg), minOutput, maxOutput, timeout);
}

float neblib::XDrive::driveTo(float x, float y, float timeout)
{
    return this->driveToPose(x, y, imu->heading(vex::rotationUnits::deg), -infinityf(), infinityf(), timeout);
}

float neblib::XDrive::driveToPose(float x, float y, float heading, float minOutput, float maxOutput, float timeout)
{
    linearPID->reset();
    rotationalPID->reset();
    float time = 0;

    while ((!linearPID->isSettled() || !rotationalPID->isSettled()) && time < timeout)
    {
        neblib::Pose currentPose = odometry->getPose();
        float linearOutput = linearPID->getOutput(hypotf(x - currentPose.x, y - currentPose.y), minOutput, maxOutput);
        float rotationalOutput = rotationalPID->getOutput(neblib::wrap(heading - imu->heading(vex::rotationUnits::deg), -180, 180));

        float driveAngle = atan2f(x - currentPose.x, y - currentPose.y);
        this->driveGlobal(linearOutput * cosf(driveAngle), linearOutput * sinf(driveAngle), rotationalOutput, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

float neblib::XDrive::driveToPose(float x, float y, float heading, float timeout)
{
    return this->driveToPose(x, y, heading, -infinityf(), infinityf(), timeout);
}