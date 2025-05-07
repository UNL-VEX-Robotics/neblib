#pragma once
#include "vex.h"

namespace neblib
{
    class TrackerWheel
    {
    private:
        enum DeviceType
        {
            V5_MOTOR,
            V5_MOTOR_GROUP,
            V5_ROTATION,
            OS_ENCODER
        };

        DeviceType type;

        union
        {
            vex::motor *motor;
            vex::motor_group *motorGroup;
            vex::rotation *rotation;
            vex::encoder *encoder;
        };

        double inchesPerDegree;

    public:
        /// @brief Creates a tracker wheel
        /// @param motor the motor that is used
        /// @param wheelDiameter the diameter of the wheel(s)
        /// @param ratio the gear ratio from wheel to motor (wheel gear teeth * motor gear teeth)
        TrackerWheel(vex::motor &motor, double wheelDiameter, double ratio = 1.0);
        /// @brief Creates a tracker wheel
        /// @param motorGroup the motor group that is used
        /// @param wheelDiameter the diameter of the wheel(s)
        /// @param ratio the gear ratio from wheel to motor group (wheel gear teeth * motor group gear teeth)
        TrackerWheel(vex::motor_group &motorGroup, double wheelDiameter, double ratio = 1.0);
        /// @brief Creates a tracker wheel
        /// @param rotation the V5 rotation sensor that is used
        /// @param wheelDiameter the diameter of the wheel(s)
        /// @param ratio the gear ratio from wheel to rotation sensor (wheel gear teeth * rotation gear teeth)
        TrackerWheel(vex::rotation &rotation, double wheelDiameter, double ratio = 1.0);
        /// @brief Creates a tracker wheel
        /// @param encoder the optical shaft encoder that is used
        /// @param wheelDiameter the diameter of the wheel(s)
        /// @param ratio the gear ratio from wheel to encoder (wheel gear teeth * encoder gear teeth)
        TrackerWheel(vex::encoder &encoder, double wheelDiameter, double ratio = 1.0);

        /// @brief Gets the position of the tracker wheel in degrees
        /// @return tracker wheel position in degrees
        double getDegrees();
        /// @brief Gets the position of the tracker wheel in inches
        /// @return tracker wheel position in inches
        double getPosition();

        /// @brief Sets the position of the tracker wheel, in degrees
        void setDegrees(double degrees);
        /// @brief Sets the position of the tracker wheel, in inches
        void setPosition(double inches);
    };
};