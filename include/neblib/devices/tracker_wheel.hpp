#pragma once

#include "vex.h"

namespace neblib
{

    /// @brief Pure virtual class for trackerwheels.
    class TrackerWheel
    {
    public:
        virtual ~TrackerWheel() = default;

        /// @brief Gets the position of the tracker wheel
        /// @return the position of the tracker wheel
        virtual double getPosition() = 0;

        /// @brief Resets the position of the tracker wheel to 0
        virtual void resetPosition() = 0;

        /// @brief Sets the position of the tracker wheel
        /// @param newPosition desired position
        /// @param units rotation unit
        virtual void setPosition(
            double newPosition,
            vex::rotationUnits units) = 0;
    };

    /// @brief Tracker wheel implementation using the VEX Rotation sensor
    class RotationTrackerWheel : public TrackerWheel
    {
    private:
        vex::rotation &rotation;
        double wheelDiameter;

    public:
        /// @brief Constructs a new RotationTrackerWheel object
        /// @param rotation VEX Rotation sensor
        /// @param wheelDiameter diameter of the trackerwheel
        RotationTrackerWheel(
            vex::rotation &rotation,
            double wheelDiameter);

        /// @brief Gets the position of the tracker wheel
        /// @return the position of the tracker wheel, in the units of the diameter
        double getPosition() override;

        /// @brief Resets the position of the tracker wheel to 0
        void resetPosition() override;

        /// @brief Sets the position of the tracker wheel
        /// @param newPosition desired position
        /// @param units rotation unit
        void setPosition(
            double newPosition,
            vex::rotationUnits units) override;
    };

} // namespace neblib