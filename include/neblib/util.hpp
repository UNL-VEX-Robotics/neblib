#pragma once
#include <functional>
#include <memory>
#include <random>
#include <cstring>
#include <cctype>
#include "vex.h"

namespace neblib
{
    /// @brief Launches a task using std::bind
    /// @tparam F
    /// @param function the function that will be running as a task
    /// @return vex::task
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

    /// @brief Determines the sign of a number
    /// @tparam T
    /// @param num a number
    /// @return 1 if positive, -1 if negative, 0 otherwise
    template <typename T>
    int sign(T num)
    {
        if (num < 0)
            return -1;
        if (num > 0)
            return 1;
        return 0;
    }

    /// @brief Converts degrees to radians
    /// @param degrees degrees
    /// @return radians
    double toRad(double degrees);

    /// @brief Converts radians to degrees
    /// @param radians radians
    /// @return degrees
    double toDeg(double radians);

    /// @brief Hard clamps a number to a range
    /// @param num number
    /// @param min minimum acceptable value
    /// @param max maximum acceptable value
    /// @return a number between min and max
    double clamp(double num, double min, double max);

    /// @brief Keeps a number within a range keeping its local value
    /// @param num number
    /// @param min minimum acceptable value
    /// @param max maximum acceptable value
    /// @return number
    double wrap(double num, double min, double max);

    /// @brief Generates a random number following the Gaussian Distribution
    /// @param mean avergae/mean
    /// @param stddev standard deviation
    /// @return random number
    double gaussRandom(double mean, double stddev);

    /// @brief Generates a random number using a uniform distribution
    /// @param min minimum acceptable value
    /// @param max maximum acceptable value
    /// @return random number
    double uniformRandom(double min, double max);

    /// @brief Determines if a string contains a substring
    /// @param str string
    /// @param substr substring
    /// @return true if string contains substring, false otherwise
    bool contains(const char *str, const char *substr);
}
