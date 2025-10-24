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
        Line(Point &p0, Point &p1);
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
}