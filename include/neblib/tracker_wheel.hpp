#pragma once
#include "vex.h"

namespace neblib
{

    class TrackerWheel
    {
    private:
        enum DeviceType
        {
            ROTATION,
            OS_ENCODER
        };

        DeviceType deviceType;

        union
        {
            vex::rotation rotation;
            vex::encoder encoder;
        };

        float wheelDiameter;

        float previousPosition = 0;

    public:
        TrackerWheel(vex::rotation &&rotation, float wheelDiameter);
        TrackerWheel(vex::encoder &&encoder, float wheelDiameter);
        ~TrackerWheel();

        float getPosition();
    };

}