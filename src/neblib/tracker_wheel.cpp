#include "neblib/tracker_wheel.hpp"

neblib::RotationTrackerWheel::RotationTrackerWheel(vex::rotation &rotation, double wheelDiameter) : rotation(rotation), wheelDiameter(wheelDiameter)
{
}

double neblib::RotationTrackerWheel::getPosition()
{
    return rotation.position(vex::rotationUnits::rev) * wheelDiameter * M_PI;
}

void neblib::RotationTrackerWheel::resetPosition()
{
    this->setPosition(0.0, vex::rotationUnits::deg);
}

void neblib::RotationTrackerWheel::setPosition(double newPosition, vex::rotationUnits units)
{
    rotation.setPosition(newPosition, units);
}

neblib::EncoderTrackerWheel::EncoderTrackerWheel(vex::encoder &encoder, double wheelDiameter) : encoder(encoder), wheelDiameter(wheelDiameter)
{
}

double neblib::EncoderTrackerWheel::getPosition()
{
    return encoder.position(vex::rotationUnits::rev) * wheelDiameter * M_PI;
}

void neblib::EncoderTrackerWheel::resetPosition()
{
    this->setPosition(0.0, vex::rotationUnits::deg);
}

void neblib::EncoderTrackerWheel::setPosition(double newPosition, vex::rotationUnits units)
{
    encoder.setPosition(newPosition, units);
}