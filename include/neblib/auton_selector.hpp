#pragma once

#include <memory>
#include <string>
#include <vector>

#include "neblib/shapes.hpp"
#include "neblib/util.hpp"
#include "vex.h"

namespace neblib
{
    /// @brief Selects an autonomous route and alliance color on the VEX Brain Screen.
    /// @details Stores route names in their supplied order. Selection starts at index 0
    ///          with the red alliance. Call run() to display the selector, then use
    ///          getRoute() and getColor() to choose the robot's autonomous behavior.
    ///          The selector does not execute routes or calibrate sensors.
    class AutonomousSelector
    {
    private:
        /// @brief Route display limit in string bytes, including the trailing "...".
        const int MAX_CHAR = 20;

        /// @brief Margin between drawables and the screen edges, in pixels.
        const int EDGE_BUFFER = 30;

        /// @brief Horizontal center of the screen, in pixels.
        const int MID_X = 240;

        /// @brief Horizontal position of the screen's right edge, in pixels.
        const int RIGHT_X = 480;

        /// @brief Outline width of all drawables, in pixels.
        const int PEN_WIDTH = 5;

        /// @brief Side length of each equilateral arrow triangle, in pixels.
        const int TRIANGLE_SIDE_LENGTH = 60;

        /// @brief Calculated triangle height, truncated to whole pixels.
        const int TRIANGLE_HEIGHT = std::sin(neblib::toRad(60.0)) * TRIANGLE_SIDE_LENGTH;

        /// @brief Vertical position of the rectangular buttons' top edges, in pixels.
        const int RECTANGLE_TOP = 120;

        /// @brief Calculated width of each rectangular button, in pixels.
        const int RECTANGLE_WIDTH = MID_X - (EDGE_BUFFER * 2);

        /// @brief Calculated height of each rectangular button, in pixels.
        const int RECTANGLE_HEIGHT = RECTANGLE_WIDTH / 2;

        /// @brief Outline color of the buttons and background of the Calibrate button.
        const vex::color OUTLINE_COLOR = vex::color(255, 255, 255);
        /// @brief Fill color of the arrow buttons.
        const vex::color TRIANGLE_COLOR = vex::color(0, 255, 0);
        /// @brief Text color of the alliance-color button.
        const vex::color TEXT_COLOR = vex::color(255, 255, 255);
        /// @brief Text color of the Calibrate button.
        const vex::color CALIBRATE_TEXT_COLOR = vex::color(75, 75, 75);
        /// @brief Fill color of the alliance-color button when red is selected.
        const vex::color RED = vex::color(255, 0, 0);
        /// @brief Fill color of the alliance-color button when blue is selected.
        const vex::color BLUE = vex::color(0, 0, 255);

        /// @brief Font used for button labels and the confirmed route display.
        const vex::fontType BUTTON_FONT = vex::fontType::mono30;
        /// @brief Font used for route names and the empty-route message.
        const vex::fontType ROUTE_FONT = vex::fontType::mono30;

        /// @brief Delay before a held touch can trigger another action, in milliseconds.
        const int RETRIGGER_MS = 500;

        /// @brief Button that selects the previous route, wrapping to the last route.
        neblib::drawable::Triangle leftArrow;
        /// @brief Button that selects the next route, wrapping to the first route.
        neblib::drawable::Triangle rightArrow;
        /// @brief Button that toggles between red and blue alliances.
        neblib::drawable::Rectangle colorButton;
        /// @brief Button that confirms the selection and ends run().
        neblib::drawable::Rectangle calibrateButton;

        /// @brief Owned route names in selection-index order.
        std::vector<std::string> routes;

        /// @brief Selected alliance color, initially vex::color::red.
        vex::color currentColor;
        /// @brief Zero-based index of the selected route, initially 0.
        int currentRoute;

        /// @brief Fills the screen with the alliance color and prints the selected route.
        /// @pre routes is nonempty and currentRoute is a valid index.
        /// @note Only updates the display; no sensors are calibrated.
        void calibrate();

    public:
        /// @brief Constructs a selector with route index 0 and the red alliance selected.
        /// @param routes Route names in the order returned by getRoute(). The selector
        ///               stores its own copy when an lvalue vector is supplied.
        /// @pre Supply at least one route before calling run().
        /// @note Construction does not draw to the screen or start a task.
        AutonomousSelector(std::vector<std::string> routes);

        /// @brief Runs the touchscreen selector until the Calibrate button is pressed.
        /// @pre routes must contain at least one name.
        /// @details Blocks the calling task while handling touches on the Brain Screen.
        ///          Arrows cycle through routes with wraparound; Color toggles the alliance.
        ///          Holding a touch repeats an action after 500 milliseconds.
        ///          Calibrate displays the chosen route on the alliance-colored background
        ///          and returns without calibrating sensors or executing a route.
        ///          Subsequent calls retain the selected route and alliance.
        /// @note Route names longer than 20 string bytes are displayed as the first 17
        ///       bytes followed by "..."; stored names and indices are unchanged.
        void run();

        /// @brief Gets the currently selected alliance color.
        /// @return vex::color::red or vex::color::blue; red before the first color change.
        vex::color getColor();

        /// @brief Gets the currently selected route index in the supplied list.
        /// @return Zero-based route index, initially 0. With a nonempty list, the index
        ///         is always less than the number of supplied routes.
        /// @note An empty list still yields 0, which does not identify a valid route.
        int getRoute();
    };
}
