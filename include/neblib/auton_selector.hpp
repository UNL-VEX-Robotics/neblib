#pragma once

#include <memory>
#include <string>
#include <vector>
#include "neblib/shapes.hpp"
#include "neblib/util.hpp"
#include "vex.h"

namespace neblib
{
    class AutonomousSelector
    {
    private:
        /// @brief Maximum characters the selector can display
        /// @todo test how many characters it can actually store
        const int MAX_CHAR = 20;

        /// @brief How close a drawable can be to the edge of the screen
        const int EDGE_BUFFER = 30;

        /// @brief The middle of the screen, x position
        const int MID_X = 240;

        /// @brief The right edge of the screen, x position
        const int RIGHT_X = 480;


        /// @brief The width of the edges of all drawables
        const int PEN_WIDTH = 5;

        /// @brief The side length of each side of each triangle
        const int TRIANGLE_SIDE_LENGTH = 60;

        /// @brief calculated height of the triangle
        const int TRIANGLE_HEIGHT = std::sin(neblib::toRad(60.0)) * TRIANGLE_SIDE_LENGTH;


        /// @brief top y position of the rectangles
        const int RECTANGLE_TOP = 120;

        /// @brief calculated width of the rectangles
        const int RECTANGLE_WIDTH = MID_X - (EDGE_BUFFER * 2);

        /// @brief calculated height of the rectangles
        const int RECTANGLE_HEIGHT = RECTANGLE_WIDTH / 2;


        // All colors
        const vex::color OUTLINE_COLOR = vex::color(255, 255, 255);
        const vex::color TRIANGLE_COLOR = vex::color(0, 255, 0);
        const vex::color TEXT_COLOR = vex::color(255, 255, 255);
        const vex::color CALIBRATE_TEXT_COLOR = vex::color(75, 75, 75);
        const vex::color RED = vex::color(255, 0, 0);
        const vex::color BLUE = vex::color(0, 0, 255);


        // All fonts
        const vex::fontType BUTTON_FONT = vex::fontType::mono30;
        const vex::fontType ROUTE_FONT = vex::fontType::mono30;


        // How long a button must be held to retrigger, milliseconds
        const int RETRIGGER_MS = 500;


        // all buttons
        neblib::drawable::Triangle leftArrow;
        neblib::drawable::Triangle rightArrow;
        neblib::drawable::Rectangle colorButton;
        neblib::drawable::Rectangle calibrateButton;

        // vector of route names
        std::vector<std::string> routes;

        // current values
        vex::color currentColor;
        int currentRoute;    

        /// @brief clears the screen and displays current route
        void calibrate();
    public:
        /// @brief Constructs an AutonomousSelector object
        /// @param routes std::vector<std::string> containing names of autonomous routes
        AutonomousSelector(std::vector<std::string> routes);

        /// @brief runs the selector on the Brain Screen
        void run();

        /// @brief Gets the color of the alliance
        /// @return vex::color storing the color of the alliance
        vex::color getColor();

        /// @brief Gets the index of the name of the route
        /// @return int representing the index of the route
        int getRoute();
    };
}