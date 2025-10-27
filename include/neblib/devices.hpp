#pragma once
#include "vex.h"
#include <vector>
#include <memory>

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

    /// @brief Basic class used to hold distance sensors
    class Ray
    {
    public:
        double yOffset;
        double xOffset;
        double headingOffset;

        /// @brief Constructs a Ray object
        /// @param xOffset x offset from center of bot
        /// @param yOffset y offset from center of bot
        /// @param headingOffset heading offset from the robot's forward, clockwise
        Ray(double xOffset, double yOffset, double headingOffset);

        /// @brief Pure virtual destructor
        virtual ~Ray() = default;

        /// @brief Pure virtual function to get the reading of the sensor
        /// @param unit distance unit
        /// @return distance
        virtual double getReading(vex::distanceUnits unit) = 0;
    };

    /// @brief Ray extension for vex distance sensors
    class Distance : public Ray
    {
    private:
        vex::distance& distance;
    
    public:
        /// @brief Constructs a distance object
        /// @param distance reference to distance sensor
        /// @param xOffset x offset from center of bot
        /// @param yOffset y offset from center of bot
        /// @param headingOffset heading offset from the robot's forward, clockwise
        Distance(vex::distance &distance, double xOffset, double yOffset, double headingOffset);

        /// @brief Gets the distance reading from the sensor
        /// @param unit distance unit
        /// @return distance
        double getReading(vex::distanceUnits unit) override;
    };

    /// @brief Class for pneumatic cylinders
    class Cylinder 
    {
    private:
        vex::digital_out cylinder;

    public:
        /// @brief Constructs a cylinder object
        /// @param port vex::triport
        Cylinder(vex::triport::port port);

        /// @brief Sets the cylinder to a desired state
        /// @param state boolean value
        void set(bool state);

        /// @brief Toggles the cylinder between states
        void toggle();

        /// @brief Gets the current state of the cylinder
        /// @return true if extended, false otherwise 
        bool getState();
    };

    /// @brief Class for a group of cylinders
    class CylinderGroup
    {
    private:
        std::vector<std::unique_ptr<vex::digital_out>> cylinders;

    public:
        /// @brief Constructs a cylinder group
        /// @param ports triports of the cylinder
        CylinderGroup(std::initializer_list<vex::triport::port> ports);

        /// @brief Constructs a cylinder group with a single cylinder
        /// @param port triport of the cylinder
        CylinderGroup(vex::triport::port port);

        /// @brief Constructs an empty cylinder group
        CylinderGroup();

        /// @brief Adds cylinders to the group
        /// @param ports ports of the cylinders
        void add(std::initializer_list<vex::triport::port> ports);

        /// @brief Adds a single cylinder to the group
        /// @param port port of the cylinder
        void add(vex::triport::port port);

        /// @brief Sets the state of all cylinders
        /// @param state desired state
        void set(bool state);

        /// @brief Toggles all cylinders between states
        void toggle();

        /// @brief Gets the state of the group
        /// @return true if extended, false otherwise
        bool getState();
    };
}