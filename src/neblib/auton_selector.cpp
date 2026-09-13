#include "neblib/auton_selector.hpp"

void neblib::AutonomousSelector::calibrate()
{
    std::string route = routes.at(currentRoute);
    if (route.length() > MAX_CHAR)
    {
        route.resize(MAX_CHAR - 3);
        route += "...";
    }
    Brain.Screen.clearScreen(currentColor);
    Brain.Screen.setPenColor(vex::color::white);
    Brain.Screen.setFillColor(currentColor);
    Brain.Screen.setFont(BUTTON_FONT);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Route: ");
    Brain.Screen.print(routes.at(currentRoute).c_str());
}

neblib::AutonomousSelector::AutonomousSelector(std::vector<std::string> routes)
    : leftArrow(
          neblib::geometry::Point(EDGE_BUFFER, (TRIANGLE_SIDE_LENGTH / 2) + EDGE_BUFFER),
          neblib::geometry::Point(TRIANGLE_HEIGHT + EDGE_BUFFER, EDGE_BUFFER),
          neblib::geometry::Point(TRIANGLE_HEIGHT + EDGE_BUFFER, EDGE_BUFFER + TRIANGLE_SIDE_LENGTH),
          PEN_WIDTH,
          OUTLINE_COLOR,
          TRIANGLE_COLOR),
      rightArrow(
          neblib::geometry::Point(RIGHT_X - EDGE_BUFFER, (TRIANGLE_SIDE_LENGTH / 2) + EDGE_BUFFER),
          neblib::geometry::Point(RIGHT_X - TRIANGLE_HEIGHT - EDGE_BUFFER, EDGE_BUFFER),
          neblib::geometry::Point(RIGHT_X - TRIANGLE_HEIGHT - EDGE_BUFFER, EDGE_BUFFER + TRIANGLE_SIDE_LENGTH),
          PEN_WIDTH,
          OUTLINE_COLOR,
          TRIANGLE_COLOR),
      colorButton(
          neblib::geometry::Point(MID_X - EDGE_BUFFER - RECTANGLE_WIDTH, RECTANGLE_TOP),
          neblib::geometry::Point(MID_X - EDGE_BUFFER, RECTANGLE_TOP + RECTANGLE_HEIGHT),
          PEN_WIDTH,
          OUTLINE_COLOR,
          RED,
          TEXT_COLOR,
          "Color",
          BUTTON_FONT),
      calibrateButton(
          neblib::geometry::Point(MID_X + EDGE_BUFFER, RECTANGLE_TOP),
          neblib::geometry::Point(MID_X + RECTANGLE_WIDTH + EDGE_BUFFER, RECTANGLE_TOP + RECTANGLE_HEIGHT),
          PEN_WIDTH,
          CALIBRATE_TEXT_COLOR,
          OUTLINE_COLOR,
          CALIBRATE_TEXT_COLOR,
          "Calibrate",
          BUTTON_FONT),
      routes(std::move(routes)),
      currentColor(vex::color::red),
      currentRoute(0)
{
}

void neblib::AutonomousSelector::run()
{
    if (routes.empty())
    {
        Brain.Screen.clearScreen(vex::color::black);
        Brain.Screen.setPenColor(vex::color::red);
        Brain.Screen.setFillColor(vex::color::black);
        Brain.Screen.setFont(ROUTE_FONT);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("No Routes");
    }
    while (true)
    {
        // Draw to the Brain Screen
        Brain.Screen.clearScreen(vex::color::black);

        rightArrow.draw();
        leftArrow.draw();
        calibrateButton.draw();
        colorButton.draw();

        Brain.Screen.setFont(ROUTE_FONT);
        Brain.Screen.setFillColor(vex::color::black);
        Brain.Screen.setPenColor(vex::color::white);
        std::string currentText = routes.at(currentRoute);
        int width = Brain.Screen.getStringWidth(currentText.c_str());
        int height = Brain.Screen.getStringHeight(currentText.c_str());
        Brain.Screen.printAt(240 - (width / 2), (TRIANGLE_SIDE_LENGTH / 2) + EDGE_BUFFER + (height / 4), currentText.c_str());

        // Wait until a button is pressed
        while (!Brain.Screen.pressing())
            vex::task::sleep(10);

        // convert Brain Screen location to neblib::geometry::Point
        neblib::geometry::Point pressedPoint(Brain.Screen.xPosition(), Brain.Screen.yPosition());

        // Button logic
        if (calibrateButton.contains(pressedPoint))
        {
            calibrate();
            break;
        }
        else if (leftArrow.contains(pressedPoint))
        {
            currentRoute--;
            if (currentRoute < 0)
                currentRoute = routes.size() - 1;
        }
        else if (rightArrow.contains(pressedPoint))
        {
            currentRoute++;
            if (currentRoute >= routes.size())
                currentRoute = 0;
        }
        else if (colorButton.contains(pressedPoint))
        {
            if (currentColor == vex::color::red)
            {
                currentColor = vex::color::blue;
                colorButton.setFillColor(BLUE);
            }
            else
            {
                currentColor = vex::color::red;
                colorButton.setFillColor(RED);
            }
        }

        // Wait until the screen is released or RETRIGGER_MS milliseconds to retrigger
        for (int i = 0; i < RETRIGGER_MS; i += 10)
        {
            if (!Brain.Screen.pressing())
                break;
            vex::task::sleep(10);
        }
    }
}

vex::color neblib::AutonomousSelector::getColor()
{
    return currentColor;
}

int neblib::AutonomousSelector::getRoute()
{
    return currentRoute;
}
