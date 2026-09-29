#pragma once

#include "vex.h"

namespace neblib
{

    class Teensy final : private vex::serial_link 
    {
    public:
        // Request Types
        enum Request : uint8_t
        {
            EXISTS = 0,

            ODOMETRY_CALIBRATE = 1,
            ODOMETRY_START = 2,
            ODOMETRY_STOP = 3,
            ODOMETRY_SET_POSE = 4,
            ODOMETRY_GET_POSE = 5
        };

    private:
        vex::mutex mutex;
    public:
        Teensy(uint32_t index);

        int32_t sendAndReceive(
            uint8_t *sendBuffer,
            int32_t sendLength,
            uint8_t *readBuffer,
            int32_t readLength,
            int32_t timeout = 500);
    };

} // Namespace neblib