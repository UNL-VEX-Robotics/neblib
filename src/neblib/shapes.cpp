#include "neblib/shapes.hpp"

neblib::drawable::Rectangle::Rectangle(
    neblib::geometry::Point p0,
    neblib::geometry::Point p1,
    int lineWidth,
    vex::color outlineColor,
    vex::color fillColor,
    vex::color textColor,
    std::string text,
    vex::fontType font)
    : p0(std::min(p0.x, p1.x), std::min(p0.y, p1.y)),
      p1(std::max(p0.x, p1.x), std::max(p0.y, p1.y)),
      lineWidth(lineWidth),
      outlineColor(outlineColor),
      fillColor(fillColor),
      textColor(textColor),
      text(text),
      font(font)
{
}

neblib::drawable::Rectangle::Rectangle(
    neblib::geometry::Point p0,
    neblib::geometry::Point p1,
    int lineWidth,
    vex::color outlineColor,
    vex::color fillColor)
    : p0(std::min(p0.x, p1.x), std::min(p0.y, p1.y)),
      p1(std::max(p0.x, p1.x), std::max(p0.y, p1.y)),
      lineWidth(lineWidth),
      outlineColor(outlineColor),
      fillColor(fillColor),
      textColor(fillColor),
      text(""),
      font(vex::fontType::mono12)
{
}

void neblib::drawable::Rectangle::setFillColor(vex::color color)
{
    fillColor = color;
}

void neblib::drawable::Rectangle::draw()
{
    Brain.Screen.setPenWidth(lineWidth);
    Brain.Screen.setPenColor(outlineColor);
    Brain.Screen.setFillColor(fillColor);

    Brain.Screen.drawRectangle(
        p0.x,
        p0.y,
        p1.x - p0.x,
        p1.y - p0.y);

    Brain.Screen.setFont(font);
    Brain.Screen.setPenColor(textColor);
    auto textWidth = Brain.Screen.getStringWidth(text.c_str());
    auto textHeight = Brain.Screen.getStringHeight(text.c_str());

    Brain.Screen.printAt(
        ((p1.x - p0.x) / 2) + p0.x - (textWidth / 2),
        ((p1.y - p0.y) / 2) + p0.y + (textHeight / 4),
        text.c_str());
}

bool neblib::drawable::Rectangle::contains(neblib::geometry::Point point)
{
    return (point.x >= p0.x) && (point.y >= p0.y) && (point.x <= p1.x) && (point.y <= p1.y);
}

neblib::drawable::Triangle::Triangle(
    neblib::geometry::Point p0,
    neblib::geometry::Point p1,
    neblib::geometry::Point p2,
    int lineWidth,
    vex::color outlineColor,
    vex::color fillColor)
    : p0(p0),
      p1(p1),
      p2(p2),
      lineWidth(lineWidth),
      outlineColor(outlineColor),
      fillColor(fillColor)
{
}

vex::color neblib::drawable::Rectangle::getFillColor()
{
    return fillColor;
}

void neblib::drawable::Triangle::setFillColor(vex::color color)
{
    fillColor = color;
}

void neblib::drawable::Triangle::draw()
{
    const int minX = static_cast<int>(
        std::ceil(std::min({p0.x, p1.x, p2.x})));
    const int maxX = static_cast<int>(
        std::floor(std::max({p0.x, p1.x, p2.x})));

    const int minY = static_cast<int>(
        std::ceil(std::min({p0.y, p1.y, p2.y})));
    const int maxY = static_cast<int>(
        std::floor(std::max({p0.y, p1.y, p2.y})));

    Brain.Screen.setPenColor(fillColor);

    for (int y = minY; y <= maxY; ++y)
    {
        for (int x = minX; x <= maxX; ++x)
        {
            if (contains(neblib::geometry::Point(x, y)))
            {
                Brain.Screen.drawPixel(x, y);
            }
        }
    }

    Brain.Screen.setPenWidth(lineWidth);
    Brain.Screen.setPenColor(outlineColor);

    Brain.Screen.drawLine(p0.x, p0.y, p1.x, p1.y);
    Brain.Screen.drawLine(p2.x, p2.y, p1.x, p1.y);
    Brain.Screen.drawLine(p0.x, p0.y, p2.x, p2.y);
}

bool neblib::drawable::Triangle::contains(neblib::geometry::Point point)
{
    const auto cross = [](neblib::geometry::Point a, neblib::geometry::Point b, neblib::geometry::Point point) -> double
    {
        return (b.x - a.x) * (point.y - a.y) - (b.y - a.y) * (point.x - a.x);
    };

    // Reject triangles with no area.
    if (cross(p0, p1, p2) == 0.0)
        return false;

    const double d0 = cross(p0, p1, point);
    const double d1 = cross(p1, p2, point);
    const double d2 = cross(p2, p0, point);

    const bool hasNegative = d0 < 0 || d1 < 0 || d2 < 0;
    const bool hasPositive = d0 > 0 || d1 > 0 || d2 > 0;

    return !(hasNegative && hasPositive);
}

vex::color neblib::drawable::Triangle::getFillColor()
{
    return fillColor;
}
