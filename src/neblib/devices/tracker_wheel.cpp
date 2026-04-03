#include "neblib/devices/tracker_wheel.hpp"

neblib::RotationTrackerWheel::RotationTrackerWheel(
    vex::rotation &rotation,
    double wheelDiameter)
    : rotation(rotation),
      wheelDiameter(wheelDiameter)
{
}

double neblib::RotationTrackerWheel::getPosition()
{
    return rotation.position(vex::rotationUnits::rev) * M_PI * wheelDiameter;
}

void neblib::RotationTrackerWheel::resetPosition()
{
    rotation.resetPosition();
}

void neblib::RotationTrackerWheel::setPosition(
    double newPosition,
    vex::rotationUnits units)
{
    rotation.setPosition(newPosition, units);
}