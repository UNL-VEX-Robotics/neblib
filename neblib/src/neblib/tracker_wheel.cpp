#include "neblib/tracker_wheel.hpp"

neblib::TrackerWheel::TrackerWheel(vex::motor &motor, double wheelDiameter, double ratio) : type(V5_MOTOR), motor(&motor), inchesPerDegree(wheelDiameter * ratio / 360) {};

neblib::TrackerWheel::TrackerWheel(vex::motor_group &motorGroup, double wheelDiameter, double ratio) : type(V5_MOTOR_GROUP), motorGroup(&motorGroup), inchesPerDegree(wheelDiameter * ratio / 360) {};

neblib::TrackerWheel::TrackerWheel(vex::rotation &&rotation, double wheelDiameter, double ratio) : type(V5_ROTATION), rotation(rotation), inchesPerDegree(wheelDiameter * ratio / 360) {};

neblib::TrackerWheel::TrackerWheel(vex::encoder &&encoder, double wheelDiameter, double ratio) : type(OS_ENCODER), encoder(encoder), inchesPerDegree(wheelDiameter * ratio / 360) {};

double neblib::TrackerWheel::getDegrees()
{
    switch (type)
    {
    case V5_MOTOR:
        return motor->position(vex::rotationUnits::deg);
    case V5_MOTOR_GROUP:
        return motorGroup->position(vex::rotationUnits::deg);
    case V5_ROTATION:
        return rotation.position(vex::rotationUnits::deg);
    case OS_ENCODER:
        return encoder.position(vex::rotationUnits::deg);
    }
    return 0.0;
}

double neblib::TrackerWheel::getPosition() { return getDegrees() * inchesPerDegree; }

void neblib::TrackerWheel::setDegrees(double degrees)
{
    switch (type)
    {
    case V5_MOTOR:
        motor->setPosition(degrees, vex::rotationUnits::deg);
    case V5_MOTOR_GROUP:
        motorGroup->setPosition(degrees, vex::rotationUnits::deg);
    case V5_ROTATION:
        rotation.setPosition(degrees, vex::rotationUnits::deg);
    case OS_ENCODER:
        encoder.setPosition(degrees, vex::rotationUnits::deg);
    }
}

void neblib::TrackerWheel::setPosition(double inches) { setDegrees((inchesPerDegree == 0) ? 0 : inches / inchesPerDegree); }