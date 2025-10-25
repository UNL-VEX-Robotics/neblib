#pragma once
#include "vex.h"
#include "neblib/devices.hpp"
#include "neblib/util.hpp"

#include <vector>

namespace neblib
{

    /// @brief Struct for holding pose of a robot (x, y, heading)
    struct Pose
    {
        double x;
        double y;
        double heading;

        /// @brief Constructs a Pose object
        /// @param x x position of a robot
        /// @param y y position of a robot
        /// @param heading heading of a robot
        Pose(double x, double y, double heading);

        Pose &operator+=(const Pose &other);
    };

    /// @brief Struct to contain a Point
    struct Point
    {
        double x;
        double y;

        /// @brief Constructs a Point
        /// @param x x position
        /// @param y y position
        Point(double x, double y);
    };

    /// @brief Struct to contain a Line
    struct Line
    {
        Point p0;
        Point p1;

        /// @brief Constructs a Line
        /// @param p0 initial point
        /// @param p1 final point
        Line(Point p0, Point p1);
    };

    /// @brief Base PositionTracking class used for pointers and references
    class PositionTracking
    {
    public:
        // Pure virtual methods that need implemented.
        virtual ~PositionTracking() = default;
        virtual neblib::Pose getPose() = 0;
        virtual void setPose(double x, double y, double heading) = 0;
    };

    /// @brief Odometry class based on 5225 E-Pilons document: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
    class Odometry : public PositionTracking
    {
    protected:
        neblib::TrackerWheel &parallel;
        neblib::TrackerWheel &perpendicular;
        vex::inertial &imu;

        double parallelOffset;
        double perpendicularOffset;

        neblib::Pose currentPose;

        double previousRotation;
        double previousParallel;
        double previousPerpendicular;

        /// @brief Gets the global change in position
        /// @return neblib::Pose containing change in x, y, and heading
        neblib::Pose getGlobalChange();

    public:
        /// @brief Creates an Odometry object
        /// @param parallelWheel any neblib::TrackerWheel representing the wheel parallel with forwards movement
        /// @param parallelOffset the offset from the turning center of the robot to the parallel wheel, in the same units as the wheel's diameter
        /// @param perpendicularWheel any neblib::TrackerWheel representing the wheel perpendicular to forwards movement
        /// @param perpendicularOffset the offset from the turning center of the robot to the perpendicular wheel, in the same units as the wheel's diameter
        /// @param imu a vex::inertial object
        Odometry(neblib::TrackerWheel &parallelWheel, double parallelOffset, neblib::TrackerWheel &perpendicularWheel, double perpendicularOffset, vex::inertial &imu);

        /// @brief Updates the internally stored pose and returns updated value
        /// @return updated neblib::Pose holding x, y, and heading
        neblib::Pose updatePose();

        /// @brief Gets the internally stored pose
        /// @return neblib::Pose holding x, y, and heading
        neblib::Pose getPose() override;

        /// @brief Sets the robot's pose to a desired pose
        /// @param x desired x position
        /// @param y desired y position
        /// @param heading desired heading value in degrees
        void setPose(double x, double y, double heading) override;

        /// @brief Calibrates the imu
        /// @param timeout a timeout before the calibration stops if not complete
        void calibrate(double timeout = infinity());
    };

    class MCL : public PositionTracking
    {
    private:
        /// @brief Struct to represent a Particle
        struct Particle
        {
            Pose pose;
            double weight;

            /// @brief Constructs a particle using a referenced position
            /// @param x desired x
            /// @param y desired y
            /// @param heading desired heading
            /// @param noise sensor noise
            Particle(double x, double y, double heading, double noise);

            /// @brief Constructs a particle in a random position
            Particle();

            /// @brief Sets the pose of the particle with noise
            /// @param x x position
            /// @param y y position
            /// @param heading heading
            /// @param noise noise
            void setPose(double x, double y, double heading, double noise);

            /// @brief Sets the particle's pose
            /// @param x x position
            /// @param y y position
            /// @param heading heading
            void setPose(double x, double y, double heading);

            /// @brief Calculates what the sensor values would be at the given particle
            /// @param obstacles std::vector of particles represented by Lines
            /// @param sensors std::vector of sensors
            /// @return std::vector<double> containing the same number of distance values as there are sensors
            std::vector<double> calculateDistances(const std::vector<Line> &obstacles, const std::vector<neblib::Ray*> &sensors);

            /// @brief Assigns the weight to the particle
            /// @param obstacles std::vector of particles represented by Lines
            /// @param sensors std::vector of sensors
            /// @param actualReadings std::vector<double> of the actual sensor readings
            /// @param stddev standard deviation
            /// @return the new weight of the particle
            double assignWeight(const std::vector<Line> &obstacles, const std::vector<neblib::Ray*> &sensors, const std::vector<double> &actualReadings, double stddev);
        };

        // devices
        std::vector<Ray*> sensors;
        std::unique_ptr<TrackerWheel> parallelWheel;
        std::unique_ptr<TrackerWheel> perpendicularWheel;
        vex::inertial &imu;

        // MCL
        std::vector<Particle> particles;
        std::vector<Line> obstacles;
        Pose estimate;
        double stddev;
        double noise;

        // Tracking
        double parallelOffset;
        double perpendicularOffset;
        double previousPerpendicular;
        double previousParallel;
        double previousRotation;

    public:
        /// @brief Creates an MCL object
        /// @param sensors vector of unique pointers to Ray objects
        /// @param parallel unique pointer to tracker wheel object
        /// @param parallelOffset parallel offset from center
        /// @param perpendicular unique pointer to tracker wheel object
        /// @param perpendicularOffset perpendicular offset from center
        /// @param imu inertial sensor
        /// @param numParticles number of particles
        /// @param obstacles vector of lines representing obstacles
        /// @param stddev standard deviation for sensor measurements
        /// @param noise added noise
        MCL(std::vector<neblib::Ray*> sensors, std::unique_ptr<TrackerWheel> parallel, double parallelOffset, std::unique_ptr<TrackerWheel> perpendicular, double perpendicularOffset, vex::inertial &imu, int numParticles, std::vector<Line> obstacles, double stddev, double noise);

        /// @brief Updates the MCL
        void update();

        /// @brief Gets the Pose estimate
        /// @return Pose
        Pose getPose() override;

        /// @brief Sets the pose of the robot
        /// @param x x position
        /// @param y y position
        /// @param heading heading
        void setPose(double x, double y, double heading) override;
    };
}