#include "neblib/util.hpp"

template <class F>
vex::task neblib::launchTask(F &&function)
{
  // static_assert(std::is_invocable_r_v<void, F>);
  return vex::task([](void *parameters)
                   {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0; }, new std::function<void()>(std::forward<F>(function)));
}

int neblib::sign(double x)
{
  return (0 < x) - (x < 0);
}

float neblib::toRad(float deg)
{
  return deg * 3.1415926535 / 180;
}

float neblib::toDeg(float rad)
{
  return rad * 180 / 3.1415926535;
}

double neblib::clamp(double num, double min, double max)
{
  if (num < min)
    return min;
  if (num > max)
    return max;
  return num;
}

float neblib::wrap(float num, float min, float max)
{
  while (num < min)
    num += (max - min);
  while (num > max)
    num -= (max - min);
  return num;
}