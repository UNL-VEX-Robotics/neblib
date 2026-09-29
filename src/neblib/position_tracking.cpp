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

neblib::SparkFunOdometry::SparkFunOdometry(
    std::shared_ptr<neblib::Coprocessor> coprocessor,
    float xOffset,
    float yOffset,
    float headingOffset)
    : coprocessor(std::move(coprocessor))
{
    uint8_t sendBuffer[] = {neblib::Coprocessor::ODOMETRY_SET_OFFSETS, // todo: separate the bytes of the floats
                            0,                                         // Byte 1 of xOffset
                            0,                                         // Byte 2 of xOffset
                            0,                                         // Byte 3 of xOffset
                            0,                                         // Byte 4 of xOffset
                            0,                                         // Byte 1 of yOffset
                            0,                                         // Byte 2 of yOffset
                            0,                                         // Byte 3 of yOffset
                            0,                                         // Byte 4 of yOffset
                            0,                                         // Byte 1 of headingOffset
                            0,                                         // Byte 2 of headingOffset
                            0,                                         // Byte 3 of headingOffset
                            0};                                        // Byte 4 of headingOffset

    this->coprocessor->send(sendBuffer, sizeof(sendBuffer), 500);
}

int neblib::SparkFunOdometry::begin()
{
    uint8_t sendBuffer[] = {neblib::Coprocessor::ODOMETRY_START};
    coprocessor->send(sendBuffer, sizeof(sendBuffer));

    return 0;
}

void neblib::SparkFunOdometry::stop()
{
    uint8_t sendBuffer[] = {neblib::Coprocessor::ODOMETRY_STOP};
    coprocessor->send(sendBuffer, sizeof(sendBuffer));
}

void neblib::SparkFunOdometry::calibrate()
{
    uint8_t sendBuffer[] = {neblib::Coprocessor::ODOMETRY_CALIBRATE};
    coprocessor->send(sendBuffer, sizeof(sendBuffer), 5000);
}

void neblib::SparkFunOdometry::setPose(neblib::Pose pose)
{
    this->setPose(pose.x, pose.y, pose.heading);
}

void neblib::SparkFunOdometry::setPose(double x, double y, double heading)
{
    float x_f = static_cast<float>(x);
    float y_f = static_cast<float>(y);
    float heading_f = static_cast<float>(heading);

    uint8_t sendBuffer[] = {neblib::Coprocessor::ODOMETRY_SET_POSE, // todo: separate the bytes of the floats
                            0,                                      // Byte 1 of x
                            0,                                      // Byte 2 of x
                            0,                                      // Byte 3 of x
                            0,                                      // Byte 4 of x
                            0,                                      // Byte 1 of y
                            0,                                      // Byte 2 of y
                            0,                                      // Byte 3 of y
                            0,                                      // Byte 4 of y
                            0,                                      // Byte 1 of heading
                            0,                                      // Byte 2 of heading
                            0,                                      // Byte 3 of heading
                            0};                                     // Byte 4 of heading
    coprocessor->send(sendBuffer, sizeof(sendBuffer));
}

neblib::Pose neblib::SparkFunOdometry::getPose()
{
    uint8_t sendBuffer[] = {neblib::Coprocessor::ODOMETRY_GET_POSE};
    uint8_t receiveBuffer[13]; // todo: Test if this needs to be larger, in theory it doesn't
    uint32_t readCount = coprocessor->sendReceive(sendBuffer, sizeof(sendBuffer), receiveBuffer, sizeof(receiveBuffer), 5);

    if (readCount != 13)
        return neblib::Pose(
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity());
    if (receiveBuffer[0] != neblib::Coprocessor::ODOMETRY_POSE) // first byte should be a verification
        return neblib::Pose(
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity());

    float x_f = 0.0f;
    float y_f = 0.0f;
    float heading_f = 0.0f;

    // todo: Read the input and assign the bytes to the float variables
    // Bytes 2-5 should b for x_f
    // Bytes 6-9 should be for y_f
    // Bytes 10-13 should be for heading_f

    return neblib::Pose(static_cast<double>(x_f),
                        static_cast<double>(y_f),
                        static_cast<double>(heading_f));
}
