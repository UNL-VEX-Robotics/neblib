#include "neblib/util.hpp"
#include "util.hpp"

template<class F>
vex::task neblib::launchTask(F&& function) {
  //static_assert(std::is_invocable_r_v<void, F>);
  return vex::task([](void* parameters) {
    std::unique_ptr<std::function<void()>> ptr{static_cast<std::function<void()>*>(parameters)};
    (*ptr)();
    return 0;
  }, new std::function<void()>(std::forward<F>(function)));
}

template <typename T>
int neblib::sign(T x) 
{
    return (T(0) < x) - (x < T(0));
}

float neblib::toRad(float deg)
{
    return deg * 3.1415926535 / 180;
}

float neblib::toDeg(float rad)
{
    return rad * 180 / 3.1415926535;
}