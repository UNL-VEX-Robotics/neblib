#include "neblib/geometry.hpp"

neblib::geometry::Point::Point(
    double x,
    double y)
    : x(x),
      y(y)
{
}

neblib::geometry::Point::Point()
    : x(0.0),
      y(0.0)
{
}
