#pragma once
#include <functional>
#include <vector>
#include "vex.h"
#include "neblib/tracker_wheel.hpp"
#include "neblib/odometry.hpp"

namespace neblib
{
    class XDrive
    {
    private:
        vex::motor_group leftFront;
        vex::motor_group rightFront;
        vex::motor_group leftBack;
        vex::motor_group rightBack;

        vex::inertial* imu;
        neblib::Odometry* odometry;

    public:
        XDrive(vex::motor_group&& leftFront, vex::motor_group&& rightFront, vex::motor_group&& leftBack, vex::motor_group&& rightBack, neblib::Odometry* odometry, vex::inertial* imu);
        XDrive(vex::motor& leftFront, vex::motor& rightFront, vex::motor& leftBack, vex::motor& rightBack, neblib::Odometry* odometry, vex::inertial* imu);

        void driveLocal(float drive, float strafe, float turn, vex::velocityUnits unit = vex::velocityUnits::pct);
        void driveLocal(float drive, float strafe, float turn, vex::voltageUnits unit);

        void drvieAngle(float velocity, float deg, float turn, vex::velocityUnits unit = vex::velocityUnits::pct);
        void driveAngle(float velocity, float deg, float turn, vex::voltageUnits unit);

        void driveGlobal(float drive, float strafe, float turn, vex::velocityUnits unit = vex::velocityUnits::pct);
        void driveGlobal(float drive, float strafe, float turn, vex::voltageUnits unit);
    };
}