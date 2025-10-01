#include "neblib/xdrive.hpp"

neblib::XDrive::XDrive(vex::motor_group &&leftFront, vex::motor_group &&rightFront, vex::motor_group &&leftBack, vex::motor_group &&rightBack, neblib::Odometry *odometry, vex::inertial *imu) : leftFront(leftFront), rightFront(rightFront), leftBack(leftBack), rightBack(rightBack), imu(imu), odometry(odometry)
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