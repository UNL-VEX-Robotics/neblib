#pragma once
#include "neblib/tracker_wheel.hpp"
#include "neblib/util.hpp"

namespace neblib
{

    /// @brief Struct for holding pose of a robot (x, y, heading)
    struct Pose
    {
        float x;
        float y;
        float heading;

        /// @brief Constructs a Pose object
        /// @param x x position of a robot
        /// @param y y position of a robot
        /// @param heading heading of a robot
        Pose(float x, float y, float heading);
    };

    /// @brief Class to track a robot's pose based on E-Pilon documentation http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
    class Odometry
    {
    private:
        neblib::TrackerWheel *parallel;
        float parallelOffset;
        neblib::TrackerWheel *perpendicular;
        float perpendicularOffset;

        vex::inertial *imu;
        float previousRotation;

        neblib::Pose pose;

        float previousParallel = 0.0;
        float previousPerpendicular = 0.0;

    public:
        /// @brief Creates an odometry object
        /// @param parallel pointer to the parallel tracker wheel object
        /// @param parallelOffset parallel tracker wheel offset from center, right is positive
        /// @param perpendicular pointer to the perpendicular tracker wheel object
        /// @param perpendicularOffset perpendicular tracker wheel offset from center, back is positive
        /// @param imu pointer to a vex inertial object
        Odometry(neblib::TrackerWheel *parallel, float parallelOffset, neblib::TrackerWheel *perpendicular, float perpendicularOffset, vex::inertial *imu);

        /// @brief Sets the pose of a robot
        /// @param x the robot's x position
        /// @param y the robot's y position
        /// @param heading the robot's heading, in deg
        void setPose(float x, float y, float heading);

        /// @brief Calibrates the inertial sensor
        void calibrate(float timeout = infinityf());

        /// @brief Gets the global change in position
        /// @return a Pose object containing change in x, y, and heading
        Pose getGlobalChange();

        /// @brief Gets the current pose of the robot
        /// @return a Pose object containing the robot's x, y, and heading
        Pose getPose();

        /// @brief Updates the robot's pose
        /// @return a Pose object containing the robot's updated x, y, and heading
        Pose updatePose();
    };

}