#include "neblib/devices/cylinder.hpp"

neblib::Cylinder::Cylinder(vex::triport::port port)
    : cylinder(port),
      state(false)
{
}

void neblib::Cylinder::setState(bool state)
{
    if (this->state == state)
        return;

    if (state)
        cylinder.off();
    else
        cylinder.on();

    this->state = state;
}

void neblib::Cylinder::toggle()
{
    setState(!state);
}

bool neblib::Cylinder::getState()
{
    return state;
}