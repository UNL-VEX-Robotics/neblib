#include "neblib/devices.hpp"

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

neblib::Ray::Ray(double xOffset, double yOffset, double headingOffset) : yOffset(yOffset), xOffset(xOffset), headingOffset(headingOffset) {}

neblib::Distance::Distance(vex::distance &distance, double xOffset, double yOffset, double headingOffset) : Ray(xOffset, yOffset, headingOffset), distance(distance) {}

double neblib::Distance::getReading(vex::distanceUnits unit) { return distance.objectDistance(unit); }

neblib::Cylinder::Cylinder(vex::triport::port port) : cylinder(port) {}

void neblib::Cylinder::set(bool state) { cylinder.set(state); }

void neblib::Cylinder::toggle() { cylinder.set(!(bool)(cylinder.value())); }

bool neblib::Cylinder::getState() { return (bool)(cylinder.value()); }

neblib::CylinderGroup::CylinderGroup(std::initializer_list<vex::triport::port> ports)
{
    for (auto p : ports)
        cylinders.push_back(std::unique_ptr<vex::digital_out>(new vex::digital_out(p)));
}

neblib::CylinderGroup::CylinderGroup(vex::triport::port port) { cylinders.push_back(std::unique_ptr<vex::digital_out>(new vex::digital_out(port))); }

neblib::CylinderGroup::CylinderGroup() {}

void neblib::CylinderGroup::add(std::initializer_list<vex::triport::port> ports)
{
    for (auto p : ports)
        cylinders.push_back(std::unique_ptr<vex::digital_out>(new vex::digital_out(p)));
}

void neblib::CylinderGroup::add(vex::triport::port port) { cylinders.push_back(std::unique_ptr<vex::digital_out>(new vex::digital_out(port))); }

void neblib::CylinderGroup::set(bool state)
{
    for (auto &c : cylinders)
        c->set(state);
}

void neblib::CylinderGroup::toggle()
{
    for (auto &c : cylinders)
        c->set(!(bool)(c->value()));
}

bool neblib::CylinderGroup::getState()
{
    if (cylinders.size() > 0)
        return (bool)(cylinders.at(0)->value());
    return false;
}