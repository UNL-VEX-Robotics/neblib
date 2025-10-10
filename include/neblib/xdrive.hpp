#pragma once
#include "vex.h"
#include "neblib/odometry.hpp"
#include "neblib/pid.hpp"

namespace neblib
{
    class XDrive
    {
    private:
        vex::motor_group frontLeft;
        vex::motor_group frontRight;
        vex::motor_group backLeft;
        vex::motor_group backRight;

        std::unique_ptr<PositionTracking> positionTracking;

        vex::inertial &imu;

        PID *turnPID;
        PID *linearPID;
        PID *rotationalPID;

    public:
        /// @brief Constructs an XDrive object
        /// @param frontLeft front left motor group
        /// @param frontRight front right motor group
        /// @param backLeft back left motor group
        /// @param backRight back right motor group
        /// @param positionTracking std::unique_ptr for position tracking
        XDrive(vex::motor_group &&frontLeft, vex::motor_group &&frontRight, vex::motor_group &&backLeft, vex::motor_group &&backRight, std::unique_ptr<PositionTracking> positionTracking, vex::inertial &imu);

        /// @brief Sets the PID used for turn-in-place movements
        /// @param pid reference to a PID object
        void setTurnPID(PID &pid);

        /// @brief Sets the PID used for the driving for move-to movements
        /// @param pid reference to a PID object
        void setLinearPID(PID &pid);

        /// @brief Sets the PID used for the turning for move-to movements
        /// @param pid reference to a PID object
        void setRotationalPID(PID &pid);

        /// @brief Robot-centric driving
        /// @param drive forward-backward component
        /// @param strafe side to side component
        /// @param turn turning component
        /// @param units velocity units
        void driveLocal(double drive, double strafe, double turn, vex::velocityUnits units = vex::velocityUnits::pct);

        /// @brief Robot-centric driving
        /// @param drive forward-backward component
        /// @param strafe side to side component
        /// @param turn turning component
        /// @param units velocity units
        void driveLocal(double drive, double strafe, double turn, vex::voltageUnits units);

        /// @brief Drives the robot at an angle relative to the robot
        /// @param velocity desired velocity
        /// @param angle angle (robot-centered) to drive at
        /// @param turn turning component
        /// @param units velocity units
        void driveAngle(double velocity, double angle, double turn, vex::velocityUnits units = vex::velocityUnits::pct);

        /// @brief Drives the robot at an angle relative to the robot
        /// @param velocity desired velocity
        /// @param angle angle (robot-centered) to drive at
        /// @param turn turning component
        /// @param units velocity units
        void driveAngle(double velocity, double angle, double turn, vex::voltageUnits units);

        /// @brief Field-centric driving
        /// @param x x component
        /// @param y y component
        /// @param turn turning component
        /// @param units velocity units
        void driveGlobal(double x, double y, double turn, vex::velocityUnits units = vex::velocityUnits::pct);

        /// @brief Field-centric driving
        /// @param x x component
        /// @param y y component
        /// @param turn turning component
        /// @param units velocity units
        void driveGlobal(double x, double y, double turn, vex::voltageUnits units);

        /// @brief Stops the motors
        /// @param stopType stop type
        void stop(vex::brakeType stopType = vex::brakeType::hold);

        /// @brief Turns for a desired number of degrees
        /// @param degrees desired degrees
        /// @param minOutput minimum speed
        /// @param maxOutput maximum speed
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double turnFor(double degrees, double minOutput, double maxOutput, double timeout = infinity());

        /// @brief Turns for a desired number of degrees
        /// @param degrees desired degrees
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double turnFor(double degrees, double timeout = infinity());

        /// @brief Turns to a desired heading
        /// @param heading desired heading
        /// @param minOutput minimum speed
        /// @param maxOutput maximum speed
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double turnTo(double heading, double minOutput, double maxOutput, double timeout = infinity());

        /// @brief Turns to a desired heading
        /// @param heading desired heading
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double turnTo(double heading, double timeout = infinity());

        /// @brief Drives to a desired coordinate
        /// @param x x coordinate
        /// @param y y coordinate
        /// @param minOutput minimum speed
        /// @param maxOutput maximum speed
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double driveTo(double x, double y, double minOutput, double maxOutput, double timeout = infinity());

        /// @brief Drives to a desired coordinate
        /// @param x x coordinate
        /// @param y y coordinate
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double driveTo(double x, double y, double timeout = infinity());

        /// @brief Drives to a desired coordinate and heading
        /// @param x x coordinate
        /// @param y y coordinate
        /// @param heading desired heading
        /// @param minOutput minimum speed
        /// @param maxOutput maximum speed
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double driveToPose(double x, double y, double heading, double minOutput, double maxOutput, double timeout = infinity());

        /// @brief Drives to a desired coordinate and heading
        /// @param x x coordinate
        /// @param y y coordinate
        /// @param heading desired heading
        /// @param timeout time before movement gives up, seconds
        /// @return time the movement took
        double driveToPose(double x, double y, double heading, double timeout = infinity());
    };
}