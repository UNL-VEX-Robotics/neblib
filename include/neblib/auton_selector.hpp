#pragma once

#include <memory>
#include <string>
#include <vector>
#include "neblib/shapes.hpp"
#include "vex.h"

namespace neblib
{
    class AutonomousSelector
    {
    private:
        const int MAX_CHAR = 50;

        class Button
        {
        private:
            std::unique_ptr<Shape> shape;
            vex::color pressedColor;
            vex::color color;

        public:
            Button(std::unique_ptr<neblib::Shape> shape, vex::color pressedColor);
            void draw();
            void setColors(vex::color fillColor, vex::color pressedColor);
            bool pressing();
        };

        Button leftArrow;
        Button rightArrow;
        Button colorButton;
        Button calibrateButton;
        
        std::vector<std::string> routes;

        vex::color currentColor;
        int currentRoute;    

        void calibrate();
    public:
        AutonomousSelector(std::vector<std::string> routes);
        void run();
    };
}