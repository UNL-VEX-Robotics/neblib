#pragma once

#include <algorithm>
#include <string>

#include "neblib/geometry.hpp"
#include "vex.h"

namespace neblib::Shapes
{

    class Shape
    {
    public: 
        virtual ~Shape() = default;

        virtual void setFillColor(vex::color color) = 0;
        virtual void draw() = 0;
        virtual bool contains(neblib::Point point) = 0;
        virtual vex::color getFillColor() = 0;
    };

    class Rectangle : public Shape
    {
    private:
        neblib::Point p0;
        neblib::Point p1;
        int lineWidth;

        vex::color outlineColor;
        vex::color fillColor;
        vex::color textColor;

        std::string text;
        int textSize;
        vex::fontType font;

    public:
        Rectangle(
            neblib::Point p0, 
            neblib::Point p1, 
            int lineWidth, 
            vex::color outlineColor, 
            vex::color fillColor, 
            vex::color textColor,
            std::string text,
            vex::fontType font);
        Rectangle(
                neblib::Point p0, 
                neblib::Point p1, 
                int lineWidth, 
                vex::color outlineColor, 
                vex::color fillColor);
        void setFillColor(vex::color color) override;
        void draw() override;
        bool contains(neblib::Point point) override;
        vex::color getFillColor() override;
    };

    class Triangle : public Shape{
    private:
        neblib::Point p0;
        neblib::Point p1;
        neblib::Point p2;
        int lineWidth;

        vex::color outlineColor;
        vex::color fillColor;

    public:
        Triangle(
            neblib::Point p0, 
            neblib::Point p1, 
            neblib::Point p2,
            int lineWidth,
            vex::color outlineColor,
            vex::color fillColor);
        void setFillColor(vex::color color) override;
        void draw() override;
        bool contains(neblib::Point point) override;
        vex::color getFillColor() override;
    };
}