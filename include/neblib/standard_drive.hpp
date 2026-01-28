#pragma once

#include "vex.h"
#include "neblib/position_tracking.hpp"
#include "neblib/pid.hpp"

namespace neblib 
{
    class StandardDrive
    {
    private:
        vex::motor_group leftMotors;
        vex::motor_group rightMotors;

        PositionTracking* positionTracking;

        TrackerWheel &parallelTrackerWheel;
        vex::inertial &imu;

        PID* turnPID;
        PID* linearPID;
        PID* angularPID;
        PID* swingPID;

    public:
        StandardDrive(vex::motor_group&& leftMotors, vex::motor_group&& rightMotors, PositionTracking* positionTracking, TrackerWheel &parallelTrackerWheel, vex::inertial &imu);

        void setTurnPID(PID* turnPID);
        void setLinearPID(PID* linearPID);
        void setAngularPID(PID* angularPID);
        void setSwingPID(PID* swingPID);

        void tankDrive(double leftInput, double rightInput, vex::velocityUnits unit = vex::velocityUnits::pct);
        void tankDrive(double leftInput, double rightInput, vex::voltageUnits unit = vex::voltageUnits::volt);
        void arcadeDrive(double linearInput, double angularInput, vex::velocityUnits unit = vex::velocityUnits::pct);
        void arcadeDrive(double linearInput, double angularInput, vex::voltageUnits unit = vex::voltageUnits::volt);

        void stop(vex::brakeType stopType = vex::brakeType::hold);

        double turnFor(double degrees, double minOutput, double maxOutput, double timeout = infinity());
        double turnFor(double degrees, double timeout = infinity());
        double turnTo(double heading, double minOutput, double maxOutput, double timeout = infinity());
        double turnTo(double heading, double timeout = infinity());

        double driveFor(double distance, double heading, double minOutput, double maxOutput, double timeout = infinity());
        double driveFor(double distance, double minOutput, double maxOutput, double timeout = infinity());
        double driveFor(double distance, double heading, double timeout = infinity());
        double driveFor(double distance, double timeout = infinity());

        double swingFor(vex::turnType direction, double degrees, double minOutput, double maxOutput, double timeout = infinity());
        double swingFor(vex::turnType direction, double degrees, double timeout = infinity());
        double swingTo(vex::turnType turnDirection, vex::directionType direction, double heading, double minOutput, double maxOutput, double timeout = infinity());
        double swingTo(vex::turnType turnDirection, vex::directionType direction, double heading, double timeout = infinity());
        double swingTo(vex::turnType turnDirection, double heading, double minOutput, double maxOutput, double timeout = infinity());
        double swingTo(vex::turnType turnDirection, double heading, double timeout = infinity());
    };
}
