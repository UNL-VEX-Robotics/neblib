#include "neblib/position_tracking.hpp"

#include <iostream>

neblib::Pose::Pose(double x, double y, double heading) : x(x), y(y), heading(heading) {}

neblib::Pose &neblib::Pose::operator+=(const neblib::Pose &other)
{
    x += other.x;
    y += other.y;
    heading = neblib::wrap(heading + other.heading, 0, 360);
    return *this;
}

neblib::Point::Point(double x, double y): x(x), y(y) {}

neblib::Line::Line(neblib::Point& p0, neblib::Point& p1): p0(p0), p1(p1) {}

neblib::Odometry::Odometry(neblib::TrackerWheel &parallelWheel, double parallelOffset, neblib::TrackerWheel &perpendicularWheel, double perpendicularOffset, vex::inertial &imu) : parallel(parallelWheel), perpendicular(perpendicularWheel), imu(imu), parallelOffset(parallelOffset), perpendicularOffset(perpendicularOffset), currentPose(0.0, 0.0, 0.0), previousRotation(0.0), previousParallel(0.0), previousPerpendicular(0.0)
{
}

neblib::Pose neblib::Odometry::getGlobalChange()
{
    double parallelPosition = parallel.getPosition();
    double perpendicularPosition = perpendicular.getPosition();

    double changeInParallel = parallelPosition - previousParallel;
    previousParallel = parallelPosition;
    double changeInPerpendicular = perpendicularPosition - previousPerpendicular;
    previousPerpendicular = perpendicularPosition;

    double currentRotation = imu.rotation(vex::rotationUnits::deg);
    double changeInRotation = neblib::toRad(currentRotation - previousRotation);

    Pose localPose = Pose(0.0, 0.0, 0.0);
    if (changeInRotation == 1e-6)
    {
        localPose.x = changeInPerpendicular;
        localPose.y = changeInParallel;
    }
    else
    {
        localPose.x = 2.0 * sin(changeInRotation / 2.0) * (changeInPerpendicular / changeInRotation + perpendicularOffset);
        localPose.y = 2.0 * sin(changeInRotation / 2.0) * (changeInParallel / changeInRotation + parallelOffset);
    }

    double averageRotation = neblib::toRad(previousRotation) + (changeInRotation / 2.0);
    previousRotation = currentRotation;

    double polarRadius = hypot(localPose.x, localPose.y);
    double polarAngle = atan2(localPose.y, localPose.x) - averageRotation;

    return Pose(polarRadius * cos(polarAngle), polarRadius * sin(polarAngle), neblib::toDeg(changeInRotation));
}

neblib::Pose neblib::Odometry::updatePose()
{
    currentPose += this->getGlobalChange();
    currentPose.heading = imu.heading(vex::rotationUnits::deg);
    return currentPose;
}

neblib::Pose neblib::Odometry::getPose()
{
    return currentPose;
}

void neblib::Odometry::setPose(double x, double y, double heading)
{
    currentPose = Pose(x, y, heading);
    imu.setHeading(heading, vex::rotationUnits::deg);
}

void neblib::Odometry::calibrate(double timeout)
{
    imu.calibrate();
    for (int mS = 0; mS < 1000 * timeout; mS += 10)
    {
        vex::task::sleep(10);
        if (imu.isCalibrating())
            break;
    }
}