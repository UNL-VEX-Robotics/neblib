#pragma once

#include <algorithm>
#include <string>

#include "neblib/geometry.hpp"
#include "vex.h"

/// @brief neblib::drawable contains shapes that can be drawn to the VEX Brain Screen
namespace neblib::drawable
{

    /// @brief Base class to implement drawable shapes
    class Shape
    {
    public:
        virtual ~Shape() = default;

        /// @brief Sets the fill or background color of the shape.
        /// @param color vex::color object representing the desired fill or background color of the shape
        virtual void setFillColor(vex::color color) = 0;

        /// @brief Draws the shape to the VEX Brain Screen.
        virtual void draw() = 0;

        /// @brief Determines if a point is within the bounds of the shape.
        /// @param point neblib::geometry::Point with an x and y coordinate
        /// @return true if the point is within the shape, false otherwise
        virtual bool contains(neblib::geometry::Point point) = 0;

        /// @brief Returns the current fill or background color of the shape.
        /// @return vex::color containing the fill or background color of the shape
        virtual vex::color getFillColor() = 0;
    };

    /// @brief A simple rectangle shape
    class Rectangle : public Shape
    {
    private:
        neblib::geometry::Point p0;
        neblib::geometry::Point p1;
        int lineWidth;

        vex::color outlineColor;
        vex::color fillColor;
        vex::color textColor;

        std::string text;
        int textSize;
        vex::fontType font;

    public:
        /// @brief Constructs a Rectangle object.
        /// @param p0 neblib::geometry::Point representing a corner of the rectangle
        /// @param p1 neblib::geometry::Point representing the opposite corner of p0
        /// @param lineWidth The desired pixel width of the outline
        /// @param outlineColor vex::color object representing the desired color of the outline
        /// @param fillColor vex::color object representing the fill or background color of the rectangle
        /// @param textColor vex::color object representing the color of the text in the rectangle
        /// @param text std::string object containing the desired text
        /// @param font vex::fontType containing the desired font of the text
        Rectangle(
            neblib::geometry::Point p0,
            neblib::geometry::Point p1,
            int lineWidth,
            vex::color outlineColor,
            vex::color fillColor,
            vex::color textColor,
            std::string text,
            vex::fontType font);

        /// @brief Constructs a Rectangle object.
        /// @param p0 neblib::geometry::Point representing a corner of the rectangle
        /// @param p1 neblib::geometry::Point representing the opposite corner of p0
        /// @param lineWidth The desired pixel width of the outline
        /// @param outlineColor vex::color object representing the desired color of the outline
        /// @param fillColor vex::color object representing the fill or background color of the rectangle
        Rectangle(
            neblib::geometry::Point p0,
            neblib::geometry::Point p1,
            int lineWidth,
            vex::color outlineColor,
            vex::color fillColor);

        /// @brief Sets the fill or background color
        /// @param color vex::color object representing the fill or background color of the rectangle
        void setFillColor(vex::color color) override;

        /// @brief Draws the rectangle on the Brain Screen
        void draw() override;

        /// @brief Determines if a point is within the bounds of the Rectangle.
        /// @param point neblib::geometry::Point with an x and y coordinate
        /// @return true if the point is within the shape, false otherwise
        bool contains(neblib::geometry::Point point) override;

        /// @brief Returns the current fill or background color of the Rectangle.
        /// @return vex::color containing the fill or background color of the Rectangle
        vex::color getFillColor() override;
    };

    /// @brief A triangle defined by 3 points
    class Triangle : public Shape
    {
    private:
        neblib::geometry::Point p0;
        neblib::geometry::Point p1;
        neblib::geometry::Point p2;
        int lineWidth;

        vex::color outlineColor;
        vex::color fillColor;

    public:
        /// @brief Constructs a Triangle object.
        /// @param p0 neblib::geometry::Point containting an x and y coordinate
        /// @param p1 neblib::geometry::Point containting an x and y coordinate
        /// @param p2 neblib::geometry::Point containting an x and y coordinate
        /// @param lineWidth The desired pixel width of the outline
        /// @param outlineColor vex::color object representing the desired color of the outline
        /// @param fillColor vex::color object representing the fill or background color of the triangle
        Triangle(
            neblib::geometry::Point p0,
            neblib::geometry::Point p1,
            neblib::geometry::Point p2,
            int lineWidth,
            vex::color outlineColor,
            vex::color fillColor);

        /// @brief Sets the fill or background color
        /// @param color vex::color object representing the fill or background color of the triangle
        void setFillColor(vex::color color) override;

        /// @brief Draws the Triangle on the Brain Screen
        void draw() override;

        /// @brief Determines if a point is within the bounds of the Triangle.
        /// @param point neblib::geometry::Point with an x and y coordinate
        /// @return true if the point is within the shape, false otherwise
        bool contains(neblib::geometry::Point point) override;

        /// @brief Returns the current fill or background color of the Triangle.
        /// @return vex::color containing the fill or background color of the Triangle
        vex::color getFillColor() override;
    };
}