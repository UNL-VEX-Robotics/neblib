#pragma once
#include "vex.h"

namespace neblib
{

    struct IMUWrapper
    {
        vex::inertial IMU;
        bool exists;

        IMUWrapper();
        IMUWrapper(vex::inertial &&IMU);
    };

}