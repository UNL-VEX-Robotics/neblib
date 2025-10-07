#pragma once
#include <functional>
#include <memory>
#include "vex.h"

namespace neblib
{
    /// @brief Creates a vex::task and allows parameters to be passed
    /// @bug couldn't do vex::task task = neblib::launchTask(std::bind(&class::func, this))
    template <class F>
    vex::task launchTask(F &&function);

    /// @brief Gets the sign of a number
    /// @param num a number
    /// @return 1 if positive, -1 if negative, 0 if 0
    int sign(double num);

    /// @brief Converts degrees to radians
    /// @param deg degrees
    /// @return radians, float
    float toRad(float deg);

    /// @brief Converts radians to degrees
    /// @param rad radians
    /// @return degrees, float
    float toDeg(float rad);

    /// @brief Wraps a number within a range, keeping the local value
    /// @param num a number
    /// @param min the minimum acceptable value
    /// @param max the maximum acceptable value
    /// @return the wrapped number, float
    float wrap(float num, float min, float max);

}
