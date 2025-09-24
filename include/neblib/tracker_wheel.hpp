#pragma once
#include "vex.h"

namespace neblib
{

    class TrackerWheel
    {
    private:
        enum Device
        {
            ROTATION,
            OS_ENCODER
        };

        Device deviceType;

        union
        {
            vex::rotation rotation;
            vex::encoder encoder;
        };

        float wheelDiameter;

        float previousPosition;

    public:
        TrackerWheel(vex::rotation &&rotation, float wheelDiameter);
        TrackerWheel(vex::encoder &&encoder, float wheelDiameter);

        float getPosition();
        float getChangeInPosition();
    };

}