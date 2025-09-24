#pragma once
#include "neblib/tracker_wheel.hpp"
#include "neblib/util.hpp"

namespace neblib
{
    struct Pose
    {
        float x;
        float y;
        float heading;

        Pose(float x, float y, float heading);
    };

    class Odometry
    {
    private:
        neblib::TrackerWheel* parallel;
        float parallelOffset;
        neblib::TrackerWheel* perpendicular;
        float perpendicularOffset;

        vex::inertial* imu;

        neblib::Pose pose;

    public:
        Odometry(neblib::TrackerWheel* parallel, float parallelOffset, neblib::TrackerWheel* perpendicular, float perpendicularOffset, vex::inertial* imu);

        void updatePosition();
        int updateTask();
    };

}