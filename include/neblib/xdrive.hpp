#pragma once

#include "neblib/control_algorithms.hpp"
#include "neblib/position_tracking.hpp"
#include "vex.h"

namespace neblib
{

    class XDrive
    {
    private:
        vex::motor_group &leftFront;
        vex::motor_group &rightFront;
        vex::motor_group &leftBack;
        vex::motor_group &rightBack;
        vex::inertial &imu;

        neblib::PositionTracking *positionTracking;

        neblib::FeedbackController *linearController;
        neblib::FeedbackController *angularController;

    public:
        /// @brief Creates a new XDrive object
        ///
        /// @param leftFront reference to a VEX motor group on the front left of the drivetrain
        /// @param rightFront reference to a VEX motor group on the front right of the drivetrain
        /// @param leftBack reference to a VEX motor group on the back left of the drivetrain
        /// @param rightBack reference to a VEX motor group on the back right of the drivetrain
        /// @param imu reference to a VEX V5 Inertial sensor
        /// @param positionTracking pointer to any neblib::PositionTracking object or nullptr
        XDrive(vex::motor_group &leftFront,
               vex::motor_group &rightFront,
               vex::motor_group &leftBack,
               vex::motor_group &rightBack,
               vex::inertial &imu,
               neblib::PositionTracking *positionTracking);

        /// @brief Sets the linear controller for the drivetrain
        ///
        /// @param linearController pointer to any neblib::FeedbackController
        void setLinearController(neblib::FeedbackController *linearController);

        /// @brief Sets the angular controller for the drivetrain
        ///
        /// @param angularController pointer to any neblib::FeedbackController
        void setAngularController(neblib::FeedbackController *angularController);

        /// @brief Drives the robot using forward, side, and turn inputs
        ///
        /// @param drive 
        /// @param strafe 
        /// @param turn 
        /// @param unit 
        void driveLocal(
            double drive,
            double strafe,
            double turn,
            vex::velocityUnits unit);

        void driveLocal(
            double drive,
            double strafe,
            double turn,
            vex::voltageUnits unit = vex::voltageUnits::volt);

        void driveAngle(
            double drive,
            double angle,
            double turn,
            vex::velocityUnits unit);

        void driveAngle(
            double drive,
            double angle,
            double turn,
            vex::voltageUnits unit = vex::voltageUnits::volt);

        void driveGlobal(
            double x,
            double y,
            double turn,
            vex::velocityUnits unit);

        void driveGlobal(
            double x,
            double y,
            double turn,
            vex::voltageUnits unit = vex::voltageUnits::volt);

        void stop(vex::brakeType brakeType = vex::brakeType::hold);

        int driveToPose(
            double x,
            double y,
            double heading,
            int timeout = infinity(),
            double minOutput = -infinity(),
            double maxOutput = infinity());

        int driveTo(
            double x,
            double y,
            int timeout = infinity(),
            double minOutput = -infinity(),
            double maxOutput = infinity());

        int turnFor(
            double degrees,
            int timeout = infinity(),
            double minOutput = -infinity(),
            double maxOutput = infinity());

        int turnTo(
            double heading,
            int timeout = infinity(),
            double minOutput = -infinity(),
            double maxOutput = infinity());
    };

} // namespace neblib