#include "neblib/utility.hpp"

template <typename T>
constexpr int neblib::sign(T val) {
    return (T(0) < val) - (val < T(0));
}
