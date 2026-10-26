#pragma once

#include "vex.h"
#include "neblib/state_machine/state_machine.hpp"
#include "neblib/state_machine/subsystem.hpp"

namespace neblib 
{
    /// @brief The Superstructure class represents the overall structure of the robot, managing its subsystems and their states.
    class Superstructure 
    {
    private:

        // Create the subsystems here //

        /// @brief A reference to the robot's time, used for timing state transitions and callbacks.
        uint32_t& robotTime;

        /// @brief Registers the states and their callbacks for the superstructure's state machine.
        void registerStates();

    public:

        /// @brief An enumeration representing the possible states of the superstructure.
        enum SuperstructureState 
        { // TODO RENAME THIS ENUM AND CREATE STATES
            IDLE,
        };

        /// @brief Constructs a Superstructure with a reference to the robot's time.
        /// @param robotTime A reference to a uint32_t variable that represents the robot's time, used for timing state transitions and callbacks.
        Superstructure(uint32_t& robotTime);

        /// @brief The state machine that manages the states and transitions for the superstructure.
        StateMachine<SuperstructureState> stateMachine;

        /// @brief Periodically updates the superstructure, handling state transitions and executing callbacks.
        void periodic();
    };
}
