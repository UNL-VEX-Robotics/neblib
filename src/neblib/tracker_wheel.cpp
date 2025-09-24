#include "neblib/tracker_wheel.hpp"

neblib::TrackerWheel::TrackerWheel(vex::rotation &&rotation, float wheelDiameter) : deviceType(ROTATION), rotation(rotation), wheelDiameter(wheelDiameter), previousPosition(0) {}

neblib::TrackerWheel::TrackerWheel(vex::encoder &&encoder, float wheelDiameter) : deviceType(OS_ENCODER), encoder(encoder), wheelDiameter(wheelDiameter), previousPosition(0) {}

float neblib::TrackerWheel::getPosition()
{
    float conversion = wheelDiameter * M_PI / 360;
    float currentPosition;
    switch (deviceType)
    {
    case ROTATION:
        currentPosition = rotation.position(vex::rotationUnits::deg) * conversion;
    case OS_ENCODER:
        currentPosition = encoder.position(vex::rotationUnits::deg) * conversion;
    }
    previousPosition = currentPosition;
    return currentPosition;
}

float neblib::TrackerWheel::getChangeInPosition()
{
    float currentPosition = this->getPosition();
    float changeInPosition = currentPosition - previousPosition;
    previousPosition = currentPosition;

    return changeInPosition;
}