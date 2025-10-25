#include "neblib/xdrive.hpp"

neblib::XDrive::XDrive(vex::motor_group &&frontLeft, vex::motor_group &&frontRight, vex::motor_group &&backLeft, vex::motor_group &&backRight, PositionTracking* positionTracking, vex::inertial &imu) : frontLeft(frontLeft), frontRight(frontRight), backLeft(backLeft), backRight(backRight), positionTracking(std::move(positionTracking)), imu(imu), turnPID(nullptr), linearPID(nullptr), rotationalPID(nullptr)
{
}

void neblib::XDrive::setTurnPID(neblib::PID& pid)
{
    turnPID = &pid;
}

void neblib::XDrive::setLinearPID(neblib::PID& pid)
{
    linearPID = &pid;
}

void neblib::XDrive::setRotationalPID(neblib::PID& pid)
{
    rotationalPID = &pid;
}

void neblib::XDrive::driveLocal(double drive, double strafe, double turn, vex::velocityUnits unit)
{
    frontLeft.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    frontRight.spin(vex::directionType::fwd, drive - strafe - turn, unit);
    backLeft.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    backRight.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::driveLocal(double drive, double strafe, double turn, vex::voltageUnits unit)
{
    frontLeft.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    frontRight.spin(vex::directionType::fwd, drive - strafe - turn, unit);
    backLeft.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    backRight.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::driveAngle(double velocity, double deg, double turn, vex::velocityUnits unit)
{
    double x = velocity * cosf(neblib::toRad(deg));
    double y = velocity * sinf(neblib::toRad(deg));

    this->driveLocal(y, x, turn, unit);
}

void neblib::XDrive::driveAngle(double velocity, double deg, double turn, vex::voltageUnits unit)
{
    double x = velocity * cosf(neblib::toRad(deg));
    double y = velocity * sinf(neblib::toRad(deg));

    this->driveLocal(y, x, turn, unit);
}

void neblib::XDrive::driveGlobal(double x, double y, double turn, vex::velocityUnits unit)
{
    this->driveAngle(hypot(x, y), neblib::toDeg(atan2f(x, y)) + imu.heading(vex::rotationUnits::deg) - 90, turn, unit);
}

void neblib::XDrive::driveGlobal(double x, double y, double turn, vex::voltageUnits unit)
{
    this->driveAngle(hypotf(x, y), neblib::toDeg(atan2f(x, y)) + imu.heading(vex::rotationUnits::deg) - 90, turn, unit);
}

void neblib::XDrive::stop(vex::brakeType stopType)
{
    frontLeft.stop(stopType);
    frontRight.stop(stopType);
    backLeft.stop(stopType);
    backRight.stop(stopType);
}

double neblib::XDrive::turnFor(double deg, double minOutput, double maxOutput, double timeout)
{
    turnPID->reset();
    double target = imu.rotation(vex::rotationUnits::deg) + deg;
    double time = 0.0;

    while (!turnPID->isSettled() && time < timeout)
    {
        double error = target - imu.rotation(vex::rotationUnits::deg);
        double output = turnPID->getOutput(error, minOutput, maxOutput);

        this->driveLocal(0.0, 0.0, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::XDrive::turnFor(double deg, double timeout)
{
    return this->turnFor(deg, -infinity(), infinity(), timeout);
}

double neblib::XDrive::turnTo(double heading, double minOutput, double maxOutput, double timeout)
{
    turnPID->reset();
    double time = 0.0;

    while (!turnPID->isSettled() && time < timeout)
    {
        double error = neblib::wrap(heading - imu.heading(vex::rotationUnits::deg), -180, 180);
        double output = turnPID->getOutput(error, minOutput, maxOutput);

        this->driveLocal(0.0, 0.0, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::XDrive::turnTo(double heading, double timeout)
{
    return this->turnTo(heading, -infinity(), infinity(), timeout);
}

double neblib::XDrive::driveTo(double x, double y, double minOutput, double maxOutput, double timeout)
{
    return this->driveToPose(x, y, imu.heading(vex::rotationUnits::deg), minOutput, maxOutput, timeout);
}

double neblib::XDrive::driveTo(double x, double y, double timeout)
{
    return this->driveToPose(x, y, imu.heading(vex::rotationUnits::deg), -infinity(), infinity(), timeout);
}

double neblib::XDrive::driveToPose(double x, double y, double heading, double minOutput, double maxOutput, double timeout)
{
    
    linearPID->reset();
    rotationalPID->reset();
    double time = 0;

    while ((!linearPID->isSettled() || !rotationalPID->isSettled()) && time < timeout)
    {
        
        neblib::Pose currentPose = positionTracking->getPose();
        double linearOutput = linearPID->getOutput(hypotf(x - currentPose.x, y - currentPose.y), minOutput, maxOutput);
        double rotationalOutput = rotationalPID->getOutput(neblib::wrap(heading - imu.heading(vex::rotationUnits::deg), -180, 180));

        double driveAngle = atan2f(y - currentPose.y, x - currentPose.x);
        this->driveGlobal(linearOutput * cosf(driveAngle), linearOutput * -sinf(driveAngle), rotationalOutput, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::XDrive::driveToPose(double x, double y, double heading, double timeout)
{
    return this->driveToPose(x, y, heading, -infinity(), infinity(), timeout);
}