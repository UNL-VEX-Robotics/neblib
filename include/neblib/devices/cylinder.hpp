#pragma once

#include "vex.h"

namespace neblib
{
    /// @brief Used as a simple interface for pneumatics
    class Cylinder
    {
    private:
        vex::led cylinder;
        bool state;

    public:
        /// @brief Constructs a new Cylinder object.
        ///
        /// @param port threewire port
        Cylinder(vex::triport::port port);

        /// @brief Sets the cylinder to a desired state
        ///
        /// @param state desired state
        void setState(bool state);

        /// @brief Toggles the state of the cylinder
        void toggle();

        /// @brief Gets the current state of the cylinder
        ///
        /// @return boolean representing state.
        bool getState();
    };

} // namespace neblib