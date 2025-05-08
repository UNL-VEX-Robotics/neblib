#pragma once

#include "vex.h"
#include "neblib/pid.hpp"
#include "neblib/tracker_wheel.hpp"

namespace neblib {

    class StandardDrive {
    private:
        vex::motor_group* leftMotorGroup;
        vex::motor_group* rightMotorGroup;
        vex::inertial inertial;

        double trackWidth;

        struct DriveConstants{
            PID::Gains gains;
            PID::ExitCondition exitCondition;
        } driveConstants;

        struct TurnConstants {
            PID::Gains gains;
            PID::ExitCondition exitCondition;
        } turnConstants;

        struct SwingConstants {
            PID::Gains gains;
            PID::ExitCondition exitCondition;
        } swingConstants;

        struct ArcConstants {
            PID::Gains gains;
            PID::ExitCondition exitCondition;
        } arcConstants;

    public:
        StandardDrive(vex::motor_group& leftMotorGroup, vex::motor_group& rightMotorGroup, vex::inertial&& inertial, double trackwidth);

        bool inertialInstalled();
        void calibrate(double heading = 0);
        bool isCalibrating();
        void setHeading(double heading);
        void setRotation(double heading);
        double getOrientation(vex::orientationType axis);
        double getHeading();
        double getRotation();

        int driveFor(double distance, int timeout, double heading, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int driveFor(double distance, int timeout, double heading, double minOutput, double maxOutput);
        int driveFor(double distance, int timeout, double minOutput, double maxOutput);
        int driveFor(double distance, int timeout, double heading);
        int driveFor(double distance, int timeout);
        int driveFor(double distance);

        int turnFor(double degrees, int timeout, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int turnFor(double degrees, int timeout, double minOutput, double maxOutput);
        int turnFor(double degrees, int timeout);
        int turnFor(double degrees);

        int turnTo(double heading, int timeout, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int turnTo(double heading, int timeout, double minOutput, double maxOutput);
        int turnTo(double heading, int timeout);
        int turnTo(double heading);

        int swingFor(vex::turnType direction, double degrees, int timeout, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int swingFor(vex::turnType direction, double degrees, int timeout, double minOutput, double maxOutput);
        int swingFor(vex::turnType direction, double degrees, int timeout);
        int swingFor(vex::turnType direction, double degrees);

        int swingTo(vex::turnType turnDirection, vex::directionType driveDirection, double heading, int timeout, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int swingTo(vex::turnType turnDirection, vex::directionType driveDirection, double heading, int timeout, double minOutput, double maxOutput);
        int swingTo(vex::turnType turnDirection, vex::directionType driveDirection, double heading, int timeout);
        int swingTo(vex::turnType turnDirection, vex::directionType driveDirection, double heading);

        int arcFor(vex::turnType direction, double radius, double degrees, int timeout, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int arcFor(vex::turnType direction, double radius, double degrees, int timeout, double minOutput, double maxOutput);
        int arcFor(vex::turnType direction, double radius, double degrees, int timeout);
        int arcFor(vex::turnType direction, double radius, double degrees);

        int arcTo(vex::turnType turnDirection, vex::directionType driveDirection, double radius, double heading, int timeout, double minOutput, double maxOutput, PID::Gains& gains, PID::ExitCondition& exitCondition);
        int arcTo(vex::turnType turnDirection, vex::directionType driveDirection, double radius, double heading, int timeout, double minOutput, double maxOutput);
        int arcTo(vex::turnType turnDirection, vex::directionType driveDirection, double radius, double heading, int timeout);
        int arcTo(vex::turnType turnDirection, vex::directionType driveDirection, double radius, double heading);
    };

};