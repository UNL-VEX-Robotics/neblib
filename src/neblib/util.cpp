#include "neblib/util.hpp"

namespace
{
    static std::mt19937 random_generator(std::random_device{}());
}

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

double neblib::gaussRandom(double mean, double stddev)
{
    std::normal_distribution<double> dist(mean, stddev);
    return dist(random_generator);
}

double neblib::uniformRandom(double min, double max)
{
    std::uniform_real_distribution<double> dist(min, max);
    return dist(random_generator);
}

bool neblib::contains(const char *str, const char *substr)
{
    if (!str || !substr)
        return false;

    size_t lenSub = std::strlen(substr);
    if (lenSub == 0)
        return true;

    for (; *str; ++str)
    {
        const char *h = str;
        const char *n = substr;

        while (*h && *n && std::tolower(static_cast<unsigned char>(*h)) == std::tolower(static_cast<unsigned char>(*n)))
        {
            ++h;
            ++n;
        }

        if (*n == '\0')
            return true;
    }

    return false;
}