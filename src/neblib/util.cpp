#include "neblib/util.hpp"

template <typename T>
int sign(T x) {
    return (T(0) < x) - (x < T(0));
}
