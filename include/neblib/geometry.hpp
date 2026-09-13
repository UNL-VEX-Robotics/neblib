#pragma once

/// @brief neblib::geometry stores geometrical objects
namespace neblib::geometry
{
    /// @brief Stores a Point along a 2D plane
    struct Point
    {
        /// @brief x-coordinate of the Point
        double x;

        /// @brief y-coordinate of the Point
        double y;

        /// @brief Constructs a Point using x and y coordinates
        /// @param x x-coordinate of the Point
        /// @param y y-coordinate of the Point
        Point(double x, double y);

        /// @brief Constructs a Point at (0,0)
        Point();
    };
}