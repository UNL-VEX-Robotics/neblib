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
        float previousRotation;

        neblib::Pose pose;

        float previousParallel = 0.0;
        float previousPerpendicular = 0.0;

    public:
        Odometry(neblib::TrackerWheel* parallel, float parallelOffset, neblib::TrackerWheel* perpendicular, float perpendicularOffset, vex::inertial* imu);

        void setPose(float x, float y, float heading);
        void calibrate();

        Pose getGlobalChange();
        Pose getPose();

        Pose updatePose();
    };

}