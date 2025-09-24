#include "neblib/tracker_wheel.hpp"

#include <iostream>

neblib::TrackerWheel::TrackerWheel(vex::rotation &&rotation, float wheelDiameter) : deviceType(ROTATION), rotation(rotation), wheelDiameter(wheelDiameter) {}

neblib::TrackerWheel::TrackerWheel(vex::encoder &&encoder, float wheelDiameter) : deviceType(OS_ENCODER), encoder(encoder), wheelDiameter(wheelDiameter) {}

neblib::TrackerWheel::~TrackerWheel()
{
    switch (deviceType)
    {
    case ROTATION:
        rotation.~rotation();
        break;
    case OS_ENCODER:
        encoder.~encoder();
        break;
    }
}

float neblib::TrackerWheel::getPosition()
{
    float currentPosition;
    switch (deviceType)
    {
    case ROTATION:
        currentPosition = rotation.position(vex::rotationUnits::rev) * wheelDiameter * M_PI;
        break;
    case OS_ENCODER:
        currentPosition = encoder.position(vex::rotationUnits::rev) * wheelDiameter * M_PI;
        break;
    }
    return currentPosition;
}