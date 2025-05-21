#pragma once
#include "vex.h"
#include "neblib/inertial_wrapper.hpp"

namespace neblib
{

    class StandardDrive
    {
    private:
        vex::motor_group leftMotorGroup;
        vex::motor_group rightMotorGroup;
        IMUWrapper IMU;

    public:
        StandardDrive(vex::motor_group &&leftMotorGroup, vex::motor_group &&rightMotorGroup, neblib::IMUWrapper &&IMU);

        void driveTank(double leftInput, double rightInput, vex::voltageUnits unit);
        void driveTank(double leftInput, double rightInput, vex::velocityUnits unit);

        void driveArcade(double driveInput, double turnInput, vex::voltageUnits unit);
        void driveArcade(double driveInput, double turnInput, vex::velocityUnits unit);
    };

}