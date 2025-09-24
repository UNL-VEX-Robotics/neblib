#pragma once
#include <functional>
#include "vex.h"

namespace neblib
{

template <class F>
vex::task launchTask(F&& function);

template <typename T>
int sign(T x);

float toRad(float deg);

float toDeg(float rad);

}
