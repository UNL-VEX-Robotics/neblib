#pragma once

#include <limits>
#include "vex.h"
#include "neblib/position_tracking.hpp"
#include "neblib/control_algorithms.hpp"

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

        int turnFor(double degrees, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int turnFor(double degrees, int timeout = std::numeric_limits<int>::max());
        int turnTo(double heading, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int turnTo(double heading, int timeout = std::numeric_limits<int>::max());

        int driveFor(double distance, double heading, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int driveFor(double distance, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int driveFor(double distance, double heading, int timeout = std::numeric_limits<int>::max());
        int driveFor(double distance, int timeout = std::numeric_limits<int>::max());

        int swingFor(vex::turnType direction, double degrees, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int swingFor(vex::turnType direction, double degrees, int timeout = std::numeric_limits<int>::max());
        int swingTo(vex::turnType turnDirection, vex::directionType direction, double heading, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int swingTo(vex::turnType turnDirection, vex::directionType direction, double heading, int timeout = std::numeric_limits<int>::max());
        int swingTo(vex::turnType turnDirection, double heading, double minOutput, double maxOutput, int timeout = std::numeric_limits<int>::max());
        int swingTo(vex::turnType turnDirection, double heading, int timeout = std::numeric_limits<int>::max());
    };
}
