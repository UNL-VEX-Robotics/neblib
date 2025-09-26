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
    };
}