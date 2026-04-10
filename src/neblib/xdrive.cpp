#include "neblib/xdrive.hpp"

neblib::XDrive::XDrive(
    vex::motor_group &leftFront,
    vex::motor_group &rightFront,
    vex::motor_group &leftBack,
    vex::motor_group &rightBack,
    vex::inertial &imu,
    neblib::PositionTracking *positionTracking)
    : leftFront(leftFront),
      rightFront(rightFront),
      leftBack(leftBack),
      rightBack(rightBack),
      imu(imu),
      positionTracking(positionTracking),
      linearController(nullptr),
      angularController(nullptr)
{
}

void neblib::XDrive::setLinearController(neblib::FeedbackController *linearController)
{
    this->linearController = linearController;
}

void neblib::XDrive::setAngularController(neblib::FeedbackController *angularController)
{
    this->angularController = angularController;
}

void neblib::XDrive::driveLocal(
    double drive,
    double strafe,
    double turn,
    vex::velocityUnits unit)
{
    leftFront.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    rightFront.spin(vex::directionType::fwd, drive - strafe - turn, unit);
    leftBack.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    rightBack.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::driveLocal(
    double drive,
    double strafe,
    double turn,
    vex::voltageUnits unit)
{
    leftFront.spin(vex::directionType::fwd, drive + strafe + turn, unit);
    rightFront.spin(vex::directionType::fwd, drive - strafe - turn, unit);
    leftBack.spin(vex::directionType::fwd, drive - strafe + turn, unit);
    rightBack.spin(vex::directionType::fwd, drive + strafe - turn, unit);
}

void neblib::XDrive::driveAngle(
    double drive,
    double angle,
    double turn,
    vex::velocityUnits unit)
{
    const double radians = neblib::toRad(angle);
    driveLocal(
        drive * cos(radians),
        drive * sin(radians),
        turn,
        unit);
}

void neblib::XDrive::driveAngle(
    double drive,
    double angle,
    double turn,
    vex::voltageUnits unit)
{
    const double radians = neblib::toRad(angle);
    driveLocal(
        drive * cos(radians),
        drive * sin(radians),
        turn,
        unit);
}

void neblib::XDrive::driveGlobal(
    double x,
    double y,
    double turn,
    vex::velocityUnits unit)
{
    driveAngle(
        hypot(y, x),
        90 - imu.heading() + neblib::toDeg(atan2(y, x)),
        turn,
        unit);
}

void neblib::XDrive::driveGlobal(
    double x,
    double y,
    double turn,
    vex::voltageUnits unit)
{
    driveAngle(
        hypot(y, x),
        90 - imu.heading() + neblib::toDeg(atan2(y, x)),
        turn,
        unit);
}

void neblib::XDrive::stop(vex::brakeType brakeType)
{
    leftFront.stop(brakeType);
    rightFront.stop(brakeType);
    leftBack.stop(brakeType);
    rightBack.stop(brakeType);
}

int neblib::XDrive::driveToPose(
    double x,
    double y,
    double heading,
    int timeout,
    double minOutput,
    double maxOutput)
{
    if (!positionTracking)
        return -1;
    if (!linearController)
        return -2;

    linearController->reset();
    if (angularController)
        angularController->reset();
    int time = 0;

    while (!linearController->isSettled() && time < timeout)
    {
        const neblib::Pose currentPose = positionTracking->getPose();
        const double drive = linearController->getOutput(
            hypot(x - currentPose.x, y - currentPose.y),
            minOutput,
            maxOutput);
        double turn = 0.0;
        if (angularController)
            turn = angularController->getOutput(
                neblib::wrap(heading - imu.heading(), -180.0, 180.0),
                minOutput,
                maxOutput);
        const double angle = atan2(y - currentPose.y, x - currentPose.x);

        driveGlobal(drive * cos(angle), drive * -sin(angle), turn, vex::voltageUnits::volt);
        time += 10;
        vex::task::sleep(10);
    }

    stop(vex::brakeType::hold);
    return time;
}

int neblib::XDrive::driveTo(
    double x,
    double y,
    int timeout,
    double minOutput,
    double maxOutput)
{
    return driveToPose(
        x,
        y,
        imu.heading(),
        timeout,
        minOutput,
        maxOutput);
}

int neblib::XDrive::turnFor(
    double degrees,
    int timeout,
    double minOutput,
    double maxOutput)
{
    if (!angularController)
        return -1;

    angularController->reset();
    int time = 0;
    double target = imu.rotation() + degrees;

    while (!angularController->isSettled() && time < timeout)
    {
        const double output = angularController->getOutput(
            target - imu.rotation(),
            minOutput,
            maxOutput);

        driveLocal(
            0.0,
            0.0,
            output,
            vex::voltageUnits::volt);

        time += 10;
        vex::task::sleep(10);
    }

    stop(vex::brakeType::hold);
    return time;
}

int neblib::XDrive::turnTo(
    double heading,
    int timeout,
    double minOutput,
    double maxOutput)
{
    if (!angularController)
        return -1;

    angularController->reset();
    int time = 0;

    while (!angularController->isSettled() && time < timeout)
    {
        const double output = angularController->getOutput(
            neblib::wrap(heading - imu.heading(), -180.0, 180.0),
            minOutput,
            maxOutput);

        driveLocal(
            0.0,
            0.0,
            output,
            vex::voltageUnits::volt);

        time += 10;
        vex::task::sleep(10);
    }

    stop(vex::brakeType::hold);
    return time;
}
