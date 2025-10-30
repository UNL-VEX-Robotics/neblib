#pragma once

#include "vex.h"
#include "neblib/util.hpp"
#include <vector>

namespace neblib
{
    /// @brief Struct to create a button on the screen
    struct Button
    {
        double x;
        double y;
        double width;
        double height;
        vex::color color;
        vex::color selectedColor;
        vex::color textColor;
        vex::color outlineColor;
        const char *text;
        bool selected;

        /// @brief Constructor for a Button
        /// @param x x position
        /// @param y y position
        /// @param width width of the button
        /// @param height height of the button
        /// @param color background color
        /// @param selectedColor background color when selected
        /// @param textColor text color
        /// @param outlineColor outline color
        /// @param text text
        Button(double x, double y, double width, double height, vex::color color, vex::color selectedColor, vex::color textColor, vex::color outlineColor, const char *text);
    };

    /// @brief Struct for a page
    struct Page
    {
        Button pageButton;
        std::vector<Button> buttons;

        /// @brief Constructs a page
        /// @param pageButton button to select the page
        /// @param buttons buttons the page contains
        Page(Button pageButton, std::initializer_list<Button> buttons);

        /// @brief Adds a button to the page
        /// @param button a button
        void addButton(Button button);

        /// @brief Adds multiple buttons to the page
        /// @param buttons buttons
        void addButtons(std::initializer_list<Button> buttons);
    };

    /// @brief Auton selector class
    class AutonSelector
    {
    private:
        vex::brain &brain;
        std::vector<Page *> pages;
        Button endButton;

        /// @brief Determines if a button is being pressed
        /// @param button the current button
        /// @return true if pressed, false otherwise
        bool buttonIsPressed(const Button &button);

        /// @brief Draws a button
        /// @param button the specified button
        void drawButton(const Button &button);

    public:
        /// @brief Constructs an auton selector
        /// @param brain the brain
        /// @param pages the pages
        /// @param endButton a button to end the selector
        AutonSelector(vex::brain &brain, std::initializer_list<Page *> pages, Button endButton);

        /// @brief Runs the selector
        void runSelector();

        /// @brief Gets the selected auton
        /// @return const char* name of the auton, nullptr if none
        const char *getAuton();

        /// @brief Gets the color of the auton
        /// @return blue if blue, red otherwise
        vex::color getColor();
    };
}