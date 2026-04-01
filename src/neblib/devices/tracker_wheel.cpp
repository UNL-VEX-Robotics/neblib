#include "neblib/devices/tracker_wheel.hpp"

neblib::RotationTrackerWheel::RotationTrackerWheel(
    vex::rotation *rotation,
    double wheelDiameter)
    : rotation(rotation),
      wheelDiameter(wheelDiameter)
{
}

double neblib::RotationTrackerWheel::getPosition()
{
    if (rotation)
        return rotation->position(vex::rotationUnits::rev) * M_PI * wheelDiameter;
    else
        return 0.0;
}

void neblib::RotationTrackerWheel::resetPosition()
{
    if (rotation)
        rotation->resetPosition();
}

void neblib::RotationTrackerWheel::setPosition(
    double newPosition,
    vex::rotationUnits units)
{
    if (rotation)
        rotation->setPosition(newPosition, units);
}