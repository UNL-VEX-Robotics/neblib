#pragma once

#include "vex.h"
#include "superstructure/superstructure.hpp"
#include "neblib/xdrive.hpp"

namespace neblib {
    class RobotContainer {
    private:

        /// @brief The superstructure of the robot, managing its subsystems and their states.
        Superstructure superstructure;

        /// @brief The XDrive drivetrain of the robot, allowing for omnidirectional movement.
        XDrive xDrive;

        /// @brief The driver controller for the robot, used for manual control.
        vex::controller driverController;

        /// @brief Configures the button bindings for the robot's controller.
        /// @details This method sets up the mappings between controller buttons and robot actions, 
        ///          allowing for manual control of the robot's subsystems and drivetrain.
        void configureButtonBindings();

    public:

        /// @brief Constructs a RobotContainer with a reference to the robot's time.
        /// @param robotTime A reference to a uint32_t variable that represents the robot's time, used for timing state transitions and callbacks.
        RobotContainer(uint32_t& robotTime);

        /// @brief Periodically updates the robot's subsystems and state machines.
        void periodic();
    };
}