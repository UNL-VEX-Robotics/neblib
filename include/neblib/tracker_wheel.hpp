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
        /// @brief Constructs a TrackerWheel object
        /// @param rotation a vex rotation object
        /// @param wheelDiameter the diameter of the tracker wheel
        TrackerWheel(vex::rotation &&rotation, float wheelDiameter);

        /// @brief Constructs a TrackerWheel object
        /// @param encoder a vex encoder object
        /// @param wheelDiameter the diameter of the tracker wheel
        TrackerWheel(vex::encoder &&encoder, float wheelDiameter);

        /// @brief Deconstructs a TrackerWheel object
        ~TrackerWheel();

        /// @brief Gets the current position of the tracker wheel
        /// @return float with the position of the tracker wheel, in the same units as the wheel diameter
        float getPosition();
    };

}