#pragma once
#include <functional>
#include <memory>
#include "vex.h"

namespace neblib
{
    template <class F>
    vex::task launchTask(F &&function)
    {
        // static_assert(std::is_invocable_r_v<void, F>);
        return vex::task([](void *parameters)
                         {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0; }, new std::function<void()>(std::forward<F>(function)));
    }

    template <typename T>
    int sign(T num)
    {
        if (num < 0)
            return -1;
        if (num > 0)
            return 1;
        return 0;
    }

    double toRad(double degrees)
    {
        return M_PI * degrees / 180.0;
    }

    double toDeg(double radians)
    {
        return radians * 180.0 / M_PI;
    }

    double clamp(double num, double min, double max)
    {
        if (num < min)
            return min;
        if (num > max)
            return max;
        return num;
    }

    double wrap(double num, double min, double max)
    {
        while (num < min)
            num += (max - min);
        while (num > max)
            num -= (max - min);
        return num;
    }

}
