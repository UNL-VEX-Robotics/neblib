#include "neblib/auton_selector.hpp"

neblib::AutonomousSelector::Button::Button(
    std::unique_ptr<neblib::Shapes::Shape> shape,
    vex::color pressedColor)
    : shape(std::move(shape)),
      pressedColor(pressedColor)
{
    color = shape->getFillColor();
}

void neblib::AutonomousSelector::Button::draw()
{
    if (pressing())
        shape->setFillColor(pressedColor);
    else
        shape->setFillColor(color);
    shape->draw();
}

void neblib::AutonomousSelector::Button::setColors(vex::color fillColor, vex::color pressedColor)
{
    this->color = fillColor;
    this->pressedColor = pressedColor;
}

bool neblib::AutonomousSelector::Button::pressing()
{
    return Brain.Screen.pressing() && shape->contains(neblib::Point(Brain.Screen.xPosition(), Brain.Screen.yPosition()));
}

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
    Brain.Screen.setFont(vex::fontType::mono30);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Route: ");
    Brain.Screen.print(routes.at(currentRoute).c_str());
}

neblib::AutonomousSelector::AutonomousSelector(std::vector<std::string> routes)
    : leftArrow(std::unique_ptr<neblib::Shapes::Shape>(new neblib::Shapes::Triangle(
                    neblib::Point(1, 50),
                    neblib::Point(85, 0),
                    neblib::Point(85, 100),
                    5,
                    vex::color(0, 0, 0),
                    vex::color(0, 255, 0))),
                vex::color(0, 175, 0)),
      rightArrow(std::unique_ptr<neblib::Shapes::Shape>(new neblib::Shapes::Triangle(
                     neblib::Point(479, 50),
                     neblib::Point(395, 0),
                     neblib::Point(395, 100),
                     5,
                     vex::color(0, 0, 0),
                     vex::color(0, 255, 0))),
                 vex::color(0, 175, 0)),
      colorButton(std::unique_ptr<neblib::Shapes::Shape>(new neblib::Shapes::Rectangle(
                      neblib::Point(20, 120),
                      neblib::Point(230, 225),
                      5,
                      vex::color::white,
                      vex::color::red,
                      vex::color::white,
                      "Color",
                      vex::fontType::mono30)),
                  vex::color(175, 0, 0)),
      calibrateButton(std::unique_ptr<neblib::Shapes::Shape>(new neblib::Shapes::Rectangle(
                          neblib::Point(250, 120),
                          neblib::Point(460, 225),
                          5,
                          vex::color(75, 75, 75),
                          vex::color::white,
                          vex::color(75, 75, 75),
                          "Calibrate",
                          vex::fontType::mono30)),
                      vex::color(230, 230, 230)),
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
        Brain.Screen.setFont(vex::fontType::mono30);
        Brain.Screen.setCursor(1, 1);
        Brain.Screen.print("No Routes");
    }
    bool calibrated = false;
    while (!calibrated)
    {
        // Draw to the Brain Screen
        Brain.Screen.clearScreen(vex::color::black);

        rightArrow.draw();
        leftArrow.draw();
        calibrateButton.draw();
        colorButton.draw();

        Brain.Screen.setFont(vex::fontType::mono40);
        Brain.Screen.setFillColor(vex::color::black);
        Brain.Screen.setPenColor(vex::color::white);
        std::string currentText = routes.at(currentRoute);
        int width = Brain.Screen.getStringWidth(currentText.c_str());
        int height = Brain.Screen.getStringHeight(currentText.c_str());
        Brain.Screen.printAt(240 - (width / 2), 50 + (height / 4), currentText.c_str());

        // Wait until a button is pressed
        while (!Brain.Screen.pressing())
            vex::task::sleep(10);

        // Button logic
        if (calibrateButton.pressing())
        {
            calibrate();
            break;
        }
        else if (leftArrow.pressing())
        {
            currentRoute--;
            if (currentRoute < 0)
                currentRoute = routes.size() - 1;
        }
        else if (rightArrow.pressing())
        {
            currentRoute++;
            if (currentRoute >= routes.size())
                currentRoute = 0;
        }
        else if (colorButton.pressing())
        {
            if (currentColor == vex::color::red)
            {
                currentColor = vex::color::blue;
                colorButton.setColors(vex::color::blue, vex::color(0, 0, 175));
            }
            else
            {
                currentColor = vex::color::red;
                colorButton.setColors(vex::color::red, vex::color(175, 0, 0));
            }
        }

        // Wait until the screen is released or 0.5 seconds to retrigger
        for (int i = 0; i < 5000; i += 10)
        {
            if (!Brain.Screen.pressing())
                break;
            vex::task::sleep(10);
        }
    }
}
