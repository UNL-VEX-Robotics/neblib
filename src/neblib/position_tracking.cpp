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

neblib::Point::Point(double x, double y) : x(x), y(y) {}

neblib::Line::Line(neblib::Point p0, neblib::Point p1) : p0(p0), p1(p1) {}

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
    if (changeInRotation <= 1e-6)
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

neblib::MCL::Particle::Particle(double x, double y, double heading, double noise) : pose(neblib::Pose(neblib::gaussRandom(x, noise), neblib::gaussRandom(y, noise), neblib::gaussRandom(heading, noise))), weight(0.0) {}

neblib::MCL::Particle::Particle() : pose(neblib::Pose(neblib::uniformRandom(-72.0, 72.0), neblib::uniformRandom(-72.0, 72.0), neblib::uniformRandom(0.0, 360.0))), weight(0.0) {}

void neblib::MCL::Particle::setPose(double x, double y, double heading, double noise) { pose = Pose(gaussRandom(x, noise), gaussRandom(y, noise), gaussRandom(heading, noise)); }

void neblib::MCL::Particle::setPose(double x, double y, double heading) { pose = Pose(x, y, heading); }

std::vector<double> neblib::MCL::Particle::calculateDistances(const std::vector<neblib::Line> &obstacles, const std::vector<neblib::Ray*> &sensors)
{
    std::vector<double> distances;
    distances.reserve(sensors.size());

    double headingRad = toRad(90.0 - pose.heading);

    for (auto &sensor : sensors)
    {
        double sensorAngle = toRad(90.0 - (pose.heading + sensor->headingOffset));
        double dx = cos(sensorAngle);
        double dy = sin(sensorAngle);
        double sensorX = pose.x + cos(headingRad) * sensor->xOffset - sin(headingRad) * sensor->yOffset;
        double sensorY = pose.y + sin(headingRad) * sensor->xOffset + cos(headingRad) * sensor->yOffset;

        double minDistance = 78.0;

        for (auto &line : obstacles)
        {
            double denom = dx * (line.p1.y - line.p0.y) - dy * (line.p1.x - line.p0.x);
            if (fabs(denom) < 1e-6)
                continue;

            double t = ((line.p0.x - sensorX) * (line.p1.y - line.p0.y) - (line.p0.y - sensorY) * (line.p1.x - line.p0.x)) / denom;
            double u = ((line.p0.x - sensorX) * dy - (line.p0.y - sensorY) * dx) / denom;

            if (t >= 1e-3 && u >= 0 && u <= 1 && t < minDistance)
                minDistance = t;
        }

        distances.push_back(minDistance);
    }
    return distances;
}

double neblib::MCL::Particle::assignWeight(const std::vector<Line> &obstacles, const std::vector<neblib::Ray*> &sensors, const std::vector<double> &actualReadings, double stddev)
{
    if (fabs(pose.x) > 72 || fabs(pose.y) > 72)
    {
        weight = 0.0;
        return weight;
    }
    else
    {
        std::vector<double> particleReadings = this->calculateDistances(obstacles, sensors);

        double logWeight = 0.0;
        for (size_t i = 0; i < particleReadings.size(); i++)
        {
            double error = particleReadings.at(i) - actualReadings.at(i);
            logWeight += -0.5 * (error / stddev) * (error / stddev);
        }

        weight = std::exp(logWeight);
        return weight;
    }
}

neblib::MCL::MCL(std::vector<neblib::Ray*> sensors, std::unique_ptr<TrackerWheel> parallel, double parallelOffset, std::unique_ptr<TrackerWheel> perpendicular, double perpendicularOffset, vex::inertial &imu, int numParticles, std::vector<Line> obstacles, double stddev, double noise) : sensors(sensors), parallelWheel(std::move(parallel)), perpendicularWheel(std::move(perpendicular)), imu(imu), obstacles(obstacles), estimate(neblib::Pose(0.0, 0.0, 0.0)), stddev(stddev), noise(noise), parallelOffset(parallelOffset), perpendicularOffset(perpendicularOffset), previousPerpendicular(0.0), previousParallel(0.0), previousRotation(0.0)
{
    particles.reserve(numParticles);
    for (int i = 0; i < numParticles; i++)
        particles.push_back(Particle());
}

void neblib::MCL::update()
{
    size_t size = particles.size();

    // Change in position
    double parallelPosition = parallelWheel->getPosition();
    double perpendiclarPosition = perpendicularWheel->getPosition();

    double changeInParallel = parallelPosition - previousParallel;
    previousParallel = parallelPosition;
    double changeInPerpendicular = perpendiclarPosition - previousPerpendicular;
    previousPerpendicular = perpendiclarPosition;

    double currentRotation = imu.rotation(vex::rotationUnits::deg);
    double changeInRotation = currentRotation - previousRotation;
    previousRotation = currentRotation;

    Pose localPose = Pose(0.0, 0.0, 0.0);
    if (changeInRotation <= 1e-6)
    {
        localPose.x = changeInPerpendicular;
        localPose.y = changeInParallel;
    }
    else
    {
        localPose.x = 2.0 * sin(changeInRotation / 2.0) * (changeInPerpendicular / changeInRotation + perpendicularOffset);
        localPose.y = 2.0 * sin(changeInRotation / 2.0) * (changeInParallel / changeInRotation + parallelOffset);
    }

    for (auto &p : particles)
    {
        double noisedChangeInRotation = changeInRotation + neblib::gaussRandom(0.0, noise);
        double particleAngle = neblib::toRad(90.0 - (p.pose.heading + noisedChangeInRotation / 2.0));
        double dxWorld = cos(particleAngle) * localPose.y - sin(particleAngle) * localPose.x + neblib::gaussRandom(0.0, noise);
        double dyWorld = sin(particleAngle) * localPose.y + cos(particleAngle) * localPose.x + neblib::gaussRandom(0.0, noise);

        p.pose.x += dxWorld;
        p.pose.y += dyWorld;
        p.pose.heading += noisedChangeInRotation;
    }

    // Assign weights
    std::vector<double> readings;
    readings.reserve(sensors.size());
    for (auto &s : sensors)
        readings.push_back(s->getReading(vex::distanceUnits::in));

    double totalWeight = 0.0;
    for (auto &p : particles)
        totalWeight += p.assignWeight(obstacles, sensors, readings, stddev);

    for (auto &p : particles)
    {
        if (totalWeight <= 1e-6)
            p.weight = 1.0 / (double)(size);
        else
            p.weight /= totalWeight;
    }

    // Resample
    double meanX = 0.0;
    double meanY = 0.0;
    for (auto &p : particles)
    {
        meanX += p.pose.x * p.weight;
        meanY += p.pose.y * p.weight;
    }

    double sinSum = 0.0;
    double cosSum = 0.0;
    for (const auto &p : particles)
    {
        double rad = neblib::toRad(p.pose.heading);
        sinSum += std::sin(rad) * p.weight;
        cosSum += std::cos(rad) * p.weight;
    }

    double meanHeading = neblib::wrap(neblib::toDeg(std::atan2(sinSum, cosSum)), 0.0, 360.0);

    estimate.x = meanX;
    estimate.y = meanY;
    estimate.heading = meanHeading;

    std::vector<double> weights(size);
    for (size_t i = 0; i < size; ++i)
        weights[i] = particles[i].weight;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<size_t> dist(weights.begin(), weights.end());

    std::vector<Particle> resampled;
    resampled.reserve(size);
    for (size_t i = 0; i < size; ++i)
    {
        size_t idx = dist(gen);
        resampled.push_back(particles[idx]);  // copy particle
        resampled.back().weight = 1.0 / size; // reset weight
    }

    particles = std::move(resampled);
}

neblib::Pose neblib::MCL::getPose() { return estimate; }

void neblib::MCL::setPose(double x, double y, double heading)
{
    for (auto &p : particles)
        p.setPose(x, y, heading, noise);
}