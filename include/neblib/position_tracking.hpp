#pragma once

#include "neblib/devices/tracker_wheel.hpp"
#include "neblib/util.hpp"
#include "vex.h"

namespace neblib
{
    /// @brief Struct to store a robot's 'Pose'
    ///
    /// x: The 'x' position of the robot
    /// y: The 'y' position of the robot
    /// heading: The orientation of the robot
    struct Pose
    {
        double x;
        double y;
        double heading;

        /// @brief Creates a new Pose object
        /// @param x x position
        /// @param y y position
        /// @param heading orientation
        Pose(
            double x,
            double y,
            double heading);

        /// @brief Creates a new Pose object
        ///
        /// Sets 'x', 'y', and 'heading' to 0.0
        Pose();
    };

    /// @brief Base class to implement position tracking algorithms
    class PositionTracking
    {
    public:
        virtual ~PositionTracking() = default;

        /// @brief Begins a self-contained loop to constantly update the Pose
        ///
        /// Designed to work best with neblib::launch_task()
        ///
        /// @return returns 0 when the loop ends
        virtual int begin() = 0;

        /// @brief Stops the self-contained update loop
        virtual void stop() = 0;

        /// @brief Calibrates the robot
        virtual void calibrate() = 0;

        /// @brief Sets the new pose of the robot
        /// @param newPose new pose of the robot
        virtual void setPose(Pose newPose) = 0;

        /// @brief Sets the new pose of the robot
        /// @param x 'x' position
        /// @param y 'y' position
        /// @param heading robot orientation
        virtual void setPose(
            double x,
            double y,
            double heading);

        /// @brief Gets the current pose of the robot
        /// @return neblib::Pose containing 'x', 'y', and orientation values
        virtual Pose getPose() = 0;
    };

    /// @brief Position tracking using arcs to approximate robot movement
    /// Based on 5225 E-Pilons document: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
    class Odometry : public PositionTracking
    {
    private:
        // ---------- Devices ----------
        neblib::TrackerWheel &parallelTrackerWheel;
        neblib::TrackerWheel &perpendicularTrackerWheel;
        vex::inertial &imu;

        // ---------- Configuration ----------
        double parallelDistance;
        double perpendicularDistance;

        // ---------- State ----------
        vex::mutex mutex;
        Pose position;
        bool running;
        double previousParallel;
        double previousPerpendicular;
        double previousRotation;

    public:
        /// @brief Constructs an Odometry object
        /// @param parallelTrackerWheel tracker wheel parallel to the forward movement of the robot
        /// @param parallelDistance distance from the turning center to the center of the wheel, right is positive
        /// @param perpendicularTrackerWheel tracker wheel perpendicular to the forward movement of the robot
        /// @param perpendicularDistance distance from the turning center to the center of the wheel, back is positive
        /// @param imu VEX V5 Inertial sensor
        Odometry(
            neblib::TrackerWheel &parallelTrackerWheel,
            double parallelDistance,
            neblib::TrackerWheel &perpendicularTrackerWheel,
            double perpendicularDistance,
            vex::inertial &imu);

        /// @brief Begins a self-contained loop to constantly update the Pose
        ///
        /// Designed to work best with neblib::launch_task()
        ///
        /// @return returns 0 when the loop ends
        int begin() override;

        /// @brief Stops the self-contained update loop
        void stop() override;

        /// @brief Calibrates the robot
        /// Blocks until the robot has calibrated
        void calibrate() override;

        /// @brief Sets the new pose of the robot
        /// @param newPose new pose of the robot
        void setPose(Pose newPose) override;

        /// @brief Sets the new pose of the robot
        /// @param x 'x' position
        /// @param y 'y' position
        /// @param heading robot orientation
        void setPose(
            double x,
            double y,
            double heading) override;

        /// @brief Gets the current pose of the robot
        /// @return neblib::Pose containing 'x', 'y', and orientation values
        Pose getPose() override;
    };

} // namespace neblib