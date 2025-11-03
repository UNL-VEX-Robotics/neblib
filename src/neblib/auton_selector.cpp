#include "neblib/auton_selector.hpp"

neblib::Button::Button(double x, double y, double width, double height, vex::color color, vex::color selectedColor, vex::color textColor, vex::color outlineColor, const char *text) : x(x), y(y), width(width), height(height), color(color), selectedColor(selectedColor), textColor(textColor), outlineColor(outlineColor), text(text), selected(false) {}

neblib::Page::Page(neblib::Button pageButton, std::initializer_list<neblib::Button> buttons) : pageButton(pageButton)
{
    for (auto &b : buttons)
        this->buttons.push_back(b);
}

void neblib::Page::addButton(neblib::Button button) { buttons.push_back(button); }

void neblib::Page::addButtons(std::initializer_list<neblib::Button> buttons)
{
    for (auto &b : buttons)
        this->buttons.push_back(b);
}

neblib::AutonSelector::AutonSelector(vex::brain &brain, std::initializer_list<neblib::Page *> pages, neblib::Button endButton) : brain(brain), endButton(endButton)
{
    for (auto &p : pages)
        this->pages.push_back(p);
}

bool neblib::AutonSelector::buttonIsPressed(const neblib::Button &button)
{
    double x = brain.Screen.xPosition();
    double y = brain.Screen.yPosition();
    return (x >= button.x && x <= button.x + button.width && y >= button.y && y <= button.y + button.height);
}

void neblib::AutonSelector::drawButton(const neblib::Button &button)
{
    // Button
    const vex::color &color = (button.selected) ? button.selectedColor : button.color;
    brain.Screen.setFillColor(color);
    brain.Screen.setPenColor(button.outlineColor);
    brain.Screen.drawRectangle(button.x, button.y, button.width, button.height);

    // Text
    int h = brain.Screen.getStringHeight(button.text);
    int w = brain.Screen.getStringWidth(button.text);
    brain.Screen.setPenColor(button.textColor);
    brain.Screen.printAt(button.x + button.width / 2 - w / 2, button.y + button.height / 2 + h / 2, button.text);
}

void neblib::AutonSelector::runSelector()
{
    while (true)
    {
        // Draw
        brain.Screen.clearScreen();
        for (auto &p : pages)
        {
            drawButton(p->pageButton);
            if (p->pageButton.selected)
            {
                for (auto &b : p->buttons)
                    drawButton(b);
            }
        }
        drawButton(endButton);
        while (!brain.Screen.pressing())
            vex::task::sleep(2);

        // Logic
        if (buttonIsPressed(endButton))
            break;

        for (auto &p : pages)
        {
            if (buttonIsPressed(p->pageButton))
            {
                for (auto &page : pages)
                    page->pageButton.selected = false;
                p->pageButton.selected = true;
            }

            if (p->pageButton.selected)
            {
                for (auto &b : p->buttons)
                {
                    if (buttonIsPressed(b))
                    {
                        for (auto &page : pages)
                        {
                            for (auto &button : page->buttons)
                                button.selected = false;
                        }
                        b.selected = true;
                    }
                }
            }
        }

        while (brain.Screen.pressing())
            vex::task::sleep(2);
    }
}

const char *neblib::AutonSelector::getAuton()
{
    for (auto &p : pages)
    {
        for (auto &b : p->buttons)
        {
            if (b.selected)
                return b.text;
        }
    }

    return nullptr;
}

vex::color neblib::AutonSelector::getColor()
{
    const char *auton = this->getAuton();
    if (neblib::contains(auton, "blue"))
        return vex::color::blue;
    else
        return vex::color::red;
}