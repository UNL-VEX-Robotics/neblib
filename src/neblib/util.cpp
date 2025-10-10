#include "neblib/util.hpp"

double neblib::toRad(double degrees)
{
    return M_PI * degrees / 180.0;
}

double neblib::toDeg(double radians)
{
    return radians * 180.0 / M_PI;
}

double neblib::clamp(double num, double min, double max)
{
    if (num < min)
        return min;
    if (num > max)
        return max;
    return num;
}

double neblib::wrap(double num, double min, double max)
{
    while (num < min)
        num += (max - min);
    while (num > max)
        num -= (max - min);
    return num;
}