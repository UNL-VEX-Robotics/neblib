#include "neblib/position_tracking.hpp"

neblib::Pose::Pose(
    double x,
    double y,
    double heading)
    : x(x),
      y(y),
      heading(heading)
{
}

neblib::Pose::Pose()
    : x(0.0),
      y(0.0),
      heading(0.0)
{
}

neblib::Odometry::Odometry(
    neblib::TrackerWheel &parallelTrackerWheel,
    double parallelDistance,
    neblib::TrackerWheel &perpendicularTrackerWheel,
    double perpendicularDistance,
    vex::inertial &imu)
    : parallelTrackerWheel(parallelTrackerWheel),
      perpendicularTrackerWheel(perpendicularTrackerWheel),
      imu(imu),
      parallelDistance(parallelDistance),
      perpendicularDistance(perpendicularDistance),
      mutex(),
      position(0.0, 0.0, 0.0),
      running(false),
      previousParallel(0.0),
      previousPerpendicular(0.0),
      previousRotation(0.0)
{
}

int neblib::Odometry::begin()
{
    running = true;
    while (running)
    {
        // ---------- Sensor Data ----------
        const double parallelPosition = parallelTrackerWheel.getPosition();
        const double perpendicularPosition = perpendicularTrackerWheel.getPosition();
        const double rotation = neblib::toRad(imu.rotation());

        // ---------- Change in Data ----------
        const double parallelChange = parallelPosition - previousParallel;
        const double perpendicularChange = perpendicularPosition - previousPerpendicular;
        const double rotationChange = rotation - previousRotation;

        // ---------- Calculate Local Position ----------
        double localX = perpendicularChange;
        double localY = parallelChange;
        if (std::abs(rotationChange) > 1e-6)
        {
            localX = 2.0 * sin(rotationChange / 2.0) * ((perpendicularChange / rotationChange) + perpendicularDistance);
            localY = 2.0 * sin(rotationChange / 2.0) * ((parallelChange / rotationChange) + parallelDistance);
        }
        const double averageRotation = previousRotation + (rotationChange / 2.0);

        // ---------- Calculate Local Polar Coordinate ----------
        const double radius = hypot(localX, localY);
        const double angle = atan2(localY, localX) - averageRotation;

        // ---------- Convert to Cartesian ----------
        const double xChange = radius * cos(angle);
        const double yChange = radius * sin(angle);

        // ---------- Update Pose ----------
        mutex.lock();
        position.x += xChange;
        position.y += yChange;
        position.heading = imu.heading();
        mutex.unlock();

        // ---------- Update Previous Values ----------
        previousParallel = parallelPosition;
        previousPerpendicular = perpendicularPosition;
        previousRotation = rotation;

        vex::task::sleep(10);
    }

    return 0;
}

void neblib::Odometry::stop()
{
    running = false;
}

void neblib::Odometry::calibrate()
{
    parallelTrackerWheel.resetPosition();
    perpendicularTrackerWheel.resetPosition();
    imu.calibrate();
    do
    {
        vex::task::sleep(5);
    } while (imu.isCalibrating());
}

void neblib::Odometry::setPose(Pose newPose)
{
    mutex.lock();
    position = newPose;
    imu.setHeading(newPose.heading, vex::rotationUnits::deg);
    imu.setRotation(newPose.heading, vex::rotationUnits::deg);
    previousRotation = neblib::toRad(newPose.heading);
    mutex.unlock();
}

void neblib::Odometry::setPose(
    double x,
    double y,
    double heading)
{
    mutex.lock();
    position = Pose(x, y, heading);
    imu.setHeading(heading, vex::rotationUnits::deg);
    imu.setRotation(heading, vex::rotationUnits::deg);
    previousRotation = neblib::toRad(heading);
    mutex.unlock();
}

neblib::Pose neblib::Odometry::getPose()
{
    mutex.lock();
    Pose copy = position;
    mutex.unlock();
    return copy;
}