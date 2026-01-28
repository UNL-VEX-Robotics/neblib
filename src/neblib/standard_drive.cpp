#include "neblib/standard_drive.hpp"

neblib::StandardDrive::StandardDrive(vex::motor_group&& leftMotors, vex::motor_group&& rightMotors, PositionTracking* positionTracking, TrackerWheel &parallelTrackerWheel, vex::inertial &imu) : leftMotors(leftMotors), rightMotors(rightMotors), positionTracking(positionTracking), parallelTrackerWheel(parallelTrackerWheel), imu(imu), turnPID(nullptr), linearPID(nullptr), angularPID(nullptr), swingPID(nullptr)
{
}

void neblib::StandardDrive::setTurnPID(PID* turnPID)
{
    this->turnPID = turnPID;
}

void neblib::StandardDrive::setLinearPID(PID* linearPID)
{
    this->linearPID = linearPID;
}

void neblib::StandardDrive::setAngularPID(PID* angularPID)
{
    this->angularPID = angularPID;
}

void neblib::StandardDrive::setSwingPID(PID* swingPID)
{
    this->swingPID = swingPID;
}

void neblib::StandardDrive::tankDrive(double leftInput, double rightInput, vex::velocityUnits unit)
{
    leftMotors.spin(vex::directionType::fwd, leftInput, unit);
    rightMotors.spin(vex::directionType::fwd, rightInput, unit);
}

void neblib::StandardDrive::tankDrive(double leftInput, double rightInput, vex::voltageUnits unit)
{
    leftMotors.spin(vex::directionType::fwd, leftInput, unit);
    rightMotors.spin(vex::directionType::fwd, rightInput, unit);
}

void neblib::StandardDrive::arcadeDrive(double linearInput, double angularInput, vex::velocityUnits unit)
{
    leftMotors.spin(vex::directionType::fwd, linearInput + angularInput, unit);
    rightMotors.spin(vex::directionType::fwd, linearInput - angularInput, unit);
}

void neblib::StandardDrive::arcadeDrive(double linearInput, double angularInput, vex::voltageUnits unit)
{
    leftMotors.spin(vex::directionType::fwd, linearInput + angularInput, unit);
    rightMotors.spin(vex::directionType::fwd, linearInput - angularInput, unit);
}

void neblib::StandardDrive::stop(vex::brakeType stopType)
{
    leftMotors.stop(stopType);
    rightMotors.stop(stopType);
}

double neblib::StandardDrive::turnFor(double degrees, double minOutput, double maxOutput, double timeout)
{
    turnPID->reset();
    double target = imu.rotation(vex::rotationUnits::deg) + degrees;
    double time = 0.0;

    while (!turnPID->isSettled() && time < timeout)
    {
        double error = target - imu.rotation(vex::rotationUnits::deg);
        double output = turnPID->getOutput(error, minOutput, maxOutput);

        leftMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        rightMotors.spin(vex::directionType::rev, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::StandardDrive::turnFor(double degrees, double timeout)
{
    return this->turnFor(degrees, -infinity(), infinity(), timeout);
}

double neblib::StandardDrive::turnTo(double heading, double minOutput, double maxOutput, double timeout)
{
    turnPID->reset();
    double time = 0.0;

    while (!turnPID->isSettled() && time < timeout)
    {
        double error = neblib::wrap(heading - imu.heading(vex::rotationUnits::deg), -180.0, 180.0);
        double output = turnPID->getOutput(error, minOutput, maxOutput);

        leftMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);
        rightMotors.spin(vex::directionType::rev, output, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::StandardDrive::turnTo(double heading, double timeout)
{
    return this->turnTo(heading, -infinity(), infinity(), timeout);
}

double neblib::StandardDrive::driveFor(double distance, double heading, double minOutput, double maxOutput, double timeout)
{
    linearPID->reset();
    angularPID->reset();

    double target = parallelTrackerWheel.getPosition() + distance;
    double time = 0.0;

    while (!linearPID->isSettled() && time < timeout)
    {
        double linearError = target - parallelTrackerWheel.getPosition();
        double angularError = neblib::wrap(heading - imu.heading(vex::rotationUnits::deg), -180, 180);
        double linearOutput = linearPID->getOutput(linearError, minOutput, maxOutput);
        double angularOutput = angularPID->getOutput(angularError, -12.0, 12.0);

        leftMotors.spin(vex::directionType::fwd, linearOutput + angularOutput, vex::voltageUnits::volt);
        rightMotors.spin(vex::directionType::fwd, linearOutput - angularOutput, vex::voltageUnits::volt);

        vex::task::sleep(10);
        time += 0.01;
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::StandardDrive::driveFor(double distance, double minOutput, double maxOutput, double timeout)
{
    return this->driveFor(distance, imu.heading(vex::rotationUnits::deg), minOutput, maxOutput, timeout);
}

double neblib::StandardDrive::driveFor(double distance, double heading, double timeout)
{
    return this->driveFor(distance, heading, -infinity(), infinity(), timeout);
}

double neblib::StandardDrive::driveFor(double distance, double timeout)
{
    return this->driveFor(distance, imu.heading(vex::rotationUnits::deg), -infinity(), infinity(), timeout);
}

double neblib::StandardDrive::swingFor(vex::turnType direction, double degrees, double minOutput, double maxOutput, double timeout)
{
    swingPID->reset();
    double time = 0.0;
    if (direction == vex::turnType::right)
    {
        double target = imu.rotation(vex::rotationUnits::deg) + degrees;

        while (!swingPID->isSettled() && time < timeout)
        {
            double error = target - imu.rotation(vex::rotationUnits::deg);
            double output = swingPID->getOutput(error, minOutput, maxOutput);

            rightMotors.stop(vex::brakeType::hold);
            leftMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            time += 0.01;
        }
    } else {
        double target = imu.rotation(vex::rotationUnits::deg) - degrees;

        while (!swingPID->isSettled() && time < timeout)
        {
            double error = imu.rotation(vex::rotationUnits::deg) - target;
            double output = swingPID->getOutput(error, minOutput, maxOutput);

            leftMotors.stop(vex::brakeType::hold);
            rightMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            time += 0.01;
        }
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::StandardDrive::swingFor(vex::turnType direction, double degrees, double timeout)
{
    return this->swingFor(direction, degrees, -infinity(), infinity(), timeout);
}

double neblib::StandardDrive::swingTo(vex::turnType turnDirection, vex::directionType direction, double heading, double minOutput, double maxOutput, double timeout)
{
    swingPID->reset();
    double time = 0.0;
    if (turnDirection == vex::turnType::right)
    {
        double lower = (direction == vex::directionType::fwd) ? 0.0 : -360.0;
        double upper = (direction == vex::directionType::fwd) ? 360.0 : 0.0;
        while (!swingPID->isSettled() && time < timeout)
        {
            double error = neblib::wrap(heading - imu.heading(vex::rotationUnits::deg), lower, upper);
            double output = swingPID->getOutput(error, minOutput, maxOutput);

            rightMotors.stop(vex::brakeType::hold);
            leftMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            time += 0.01;
        }
    } else {
        double lower = (direction == vex::directionType::fwd) ? 0.0 : -360.0;
        double upper = (direction == vex::directionType::fwd) ? 360.0 : 0.0;
        while (!swingPID->isSettled() && time < timeout)
        {
            double error = neblib::wrap(imu.heading(vex::rotationUnits::deg) - heading, lower, upper);
            double output = swingPID->getOutput(error, minOutput, maxOutput);

            leftMotors.stop(vex::brakeType::hold);
            rightMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            time += 0.01;
        }
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::StandardDrive::swingTo(vex::turnType turnDirection, vex::directionType direction, double heading, double timeout)
{
    return this->swingTo(turnDirection, direction, heading, -infinity(), infinity(), timeout);
}

double neblib::StandardDrive::swingTo(vex::turnType turnDirection, double heading, double minOutput, double maxOutput, double timeout)
{
    swingPID->reset();
    double time = 0.0;
    if (turnDirection == vex::turnType::right)
    {
        while (!swingPID->isSettled() && time < timeout)
        {
            double error = neblib::wrap(heading - imu.heading(vex::rotationUnits::deg), -180.0, 180.0);
            double output = swingPID->getOutput(error, minOutput, maxOutput);

            rightMotors.stop(vex::brakeType::hold);
            leftMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            time += 0.01;
        }
    } else {
        while (!swingPID->isSettled() && time < timeout)
        {
            double error = neblib::wrap(imu.heading(vex::rotationUnits::deg) - heading, -180.0, 180.0);
            double output = swingPID->getOutput(error, minOutput, maxOutput);

            leftMotors.stop(vex::brakeType::hold);
            rightMotors.spin(vex::directionType::fwd, output, vex::voltageUnits::volt);

            vex::task::sleep(10);
            time += 0.01;
        }
    }

    this->stop(vex::brakeType::hold);

    return time;
}

double neblib::StandardDrive::swingTo(vex::turnType turnDirection, double heading, double timeout)
{
    return this->swingTo(turnDirection, heading, -infinity(), infinity(), timeout);
}