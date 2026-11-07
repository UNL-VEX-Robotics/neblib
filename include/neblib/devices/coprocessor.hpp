#pragma once

#include "vex.h"

namespace neblib
{

    class Coprocessor final : private vex::serial_link 
    {
    public:
        // Request Types
        enum Request : uint8_t
        {
            PING = 0xAA, // general

            ODOMETRY_SET_OFFSETS = 1, // Odometry
            ODOMETRY_CALIBRATE = 2,
            ODOMETRY_START = 3,
            ODOMETRY_STOP = 4,
            ODOMETRY_SET_POSE = 5, // Expects 3 floats, X, Y, Heading (degrees) | 12 bytes
            ODOMETRY_GET_POSE = 6,
            ODOMETRY_SET_LINEAR_SCALAR = 7, // Expects 1 float, 0.872-1.127 | 4 bytes
            ODOMETRY_SET_ANGULAR_SCALAR = 8, // Expects 1 float, 0.872-1.127 | 4 bytes

        };

        // Result Types
        enum Result : uint8_t
        {
            ERROR = 0, // general
            PING_SUCCESS = 0x55, // ping response
            SUCCESS = 0x67,

            ODOMETRY_POSE = 2, // Odometry
        };

    private:
        vex::mutex mutex; // prevents race conditions
    public:
        /// @brief Constructs a Coprocessor Object
        /// @param index The port that the Coprocessor uses
        Coprocessor(uint32_t index);

        /// @brief Sends a request and waits for a response. Wait time is determined by timeout.
        /// @param sendBuffer byte array that is being sent
        /// @param sendLength length of the byte array
        /// @param readBuffer byte array that the response will be read into
        /// @param readLength length of the byte array
        /// @param timeout the amount of time to wait for a response, milliseconds
        /// @return The number of bytes received, or -1 if the send wasn't completed.
        int32_t sendReceive(
            uint8_t *sendBuffer,
            int32_t sendLength,
            uint8_t *readBuffer,
            int32_t readLength,
            int32_t timeout = 500);

        /// @brief Waits for incoming data without sending a request. Used for reading a continuous datastream.
        /// @param readBuffer byte array that the data will be read into
        /// @param readLength length of the byte array
        /// @param timeout the amount of time to wait for data, milliseconds
        /// @return The number of bytes received
        int32_t receive(uint8_t *readBuffer, int32_t readLength, int32_t timeout = 500);

        /// @brief Sends PING request, waits for a response
        /// @param timeout the amount of time to wait for a response, milliseconds
        /// @return true if response is SUCCESS, false otherwise
        bool ping(int32_t timeout = 500);

        /// @brief Sends and waits for acknoledgement
        /// @param sendBuffer byte array that is being sent
        /// @param sendLength length of the byte array
        /// @param timeout the amount of time to wait for validation, milliseconds
        /// @return True if successfully sent, false otherwise
        bool send(uint8_t *sendBuffer, int32_t sendLength, int32_t timeout = 5);
    };

} // Namespace neblib