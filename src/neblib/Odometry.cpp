#include "neblib/odometry.hpp"

#include <iostream>

neblib::Pose::Pose(float x, float y, float heading) : x(x), y(y), heading(heading) {}

neblib::Odometry::Odometry(neblib::TrackerWheel *parallel, float parallelOffset, neblib::TrackerWheel *perpendicular, float perpendicularOffset, vex::inertial *imu) : parallel(parallel), parallelOffset(parallelOffset), perpendicular(perpendicular), perpendicularOffset(perpendicularOffset), imu(imu), previousRotation(0), pose(Pose(0, 0, 0)) {}

void neblib::Odometry::setPose(float x, float y, float heading)
{
    pose = Pose(x, y, heading);
    previousRotation = heading;
    imu->setHeading(heading, vex::rotationUnits::deg);
    imu->setRotation(heading, vex::rotationUnits::deg);
}

void neblib::Odometry::calibrate(float timeout)
{
    float t = 0;
    imu->calibrate();
    do
    {
        vex::task::sleep(10);
        t += 0.01;
    } while (imu->isCalibrating() && t < timeout);
    
    previousRotation = pose.heading;
    imu->setHeading(pose.heading, vex::rotationUnits::deg);
    imu->setRotation(pose.heading, vex::rotationUnits::deg);
}

neblib::Pose neblib::Odometry::updatePose()
{
    Pose globalChange = this->getGlobalChange();
    pose.x += globalChange.x;
    pose.y += globalChange.y;
    pose.heading = imu->heading(vex::rotationUnits::deg);

    return pose;
}

neblib::Pose neblib::Odometry::getGlobalChange()
{
    float parallelPosition = parallel->getPosition();
    float perpendicularPosition = perpendicular->getPosition();

    float changeInParallel = parallelPosition - previousParallel;
    previousParallel = parallelPosition;
    float changeInPerpendicular = perpendicularPosition - previousPerpendicular;
    previousPerpendicular = perpendicularPosition;

    float currentRotation = imu->rotation(vex::rotationUnits::deg);
    float changeInRotation = neblib::toRad(currentRotation - previousRotation);

    Pose localPose = Pose(0, 0, 0);
    if (changeInRotation == 0.0)
    {
        localPose.x = changeInPerpendicular;
        localPose.y = changeInParallel;
    }
    else
    {
        localPose.x = 2.0 * sinf(changeInRotation / 2.0) * (changeInPerpendicular / changeInRotation + perpendicularOffset);
        localPose.y = 2.0 * sinf(changeInRotation / 2.0) * (changeInParallel / changeInRotation + parallelOffset);
    }

    float averageRotation = neblib::toRad(previousRotation) + changeInRotation / 2.0;
    previousRotation = currentRotation;

    float polarRadius = sqrtf(powf(localPose.x, 2.0) + powf(localPose.y, 2.0));
    float polarAngle = atan2f(localPose.y, localPose.x) - averageRotation;

    return Pose(polarRadius * cosf(polarAngle), polarRadius * sinf(polarAngle), neblib::toDeg(changeInRotation));
}

neblib::Pose neblib::Odometry::getPose() { return pose; }