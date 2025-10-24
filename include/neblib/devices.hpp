#pragma once
#include "vex.h"

namespace neblib
{
    /// @brief Base TrackerWheel class used for pointers and references
    class TrackerWheel
    {
    public:
        /// @brief Deconstructs a TrackerWheel object
        virtual ~TrackerWheel() = default;

        // Pure virtual methods that must be implemented
        virtual double getPosition() = 0;
        virtual void resetPosition() = 0;
        virtual void setPosition(double newPosition, vex::rotationUnits units) = 0;
    };

    /// @brief TrackerWheel class with a vex v5 rotation sensor
    class RotationTrackerWheel : public TrackerWheel
    {
    private:
        vex::rotation &rotation;
        double wheelDiameter;

    public:
        /// @brief Constructs a RotationTrackerWheel object
        /// @param rotation a vex::rotation object
        /// @param wheelDiameter the diameter of the tracker wheel
        RotationTrackerWheel(vex::rotation &rotation, double wheelDiameter);

        /// @brief Gets the position of the tracker wheel, unit = wheelDiameter unit
        /// @return The position of the tracker wheel
        double getPosition() override;

        /// @brief Resets the position of the rotation sensor to zero
        void resetPosition() override;

        /// @brief Sets the position of the rotation sensor to a desired position
        /// @param newPosition desired position
        /// @param units vex::rotationUnits
        void setPosition(double newPosition, vex::rotationUnits units) override;
    };

    /// @brief TrackerWheel class with a vex optical shaft encoder
    class EncoderTrackerWheel : public TrackerWheel
    {
    private:
        vex::encoder &encoder;
        double wheelDiameter;

    public:
        /// @brief Creates an EncoderTrackerWheel object
        /// @param encoder a vex::encoder object
        /// @param wheelDiameter the diameter of the tracker wheel
        EncoderTrackerWheel(vex::encoder &encoder, double wheelDiameter);

        /// @brief Gets the position of the tracker wheel, unit = wheelDiameter unit
        /// @return The position of the tracker wheel
        double getPosition() override;

        /// @brief Resets the position of the encoder to zero
        void resetPosition() override;

        /// @brief Sets the position of the encoder to a desired value
        /// @param newPosition desired position
        /// @param units vex::rotationUnits
        void setPosition(double newPosition, vex::rotationUnits units) override;
    };

    class Ray
    {
    public:
        double yOffset;
        double xOffset;
        double headingOffset;

        Ray(double xOffset, double yOffset, double headingOffset);
        virtual ~Ray() = default;

        virtual double getReading(vex::distanceUnits unit) = 0;
    };

    class Distance : public Ray
    {
    private:
        vex::distance& distance;
    
    public:
        Distance(vex::distance &distance, double xOffset, double yOffset, double headingOffset);

        double getReading(vex::distanceUnits unit) override;
    };
}