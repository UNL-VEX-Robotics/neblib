#pragma once

#include "vex.h"
#include "state_machine.hpp"
#include <cstdint>

namespace neblib 
{
    /// @brief A base class for creating subsystems with their own state machines.
    /// @tparam StateEnum The enum type representing the states of the subsystem.
    template <typename StateEnum>
    class Subsystem 
    {
    protected:
        /// @brief A reference to the robot's time, used for timing state transitions and callbacks.
        uint32_t& robotTime;
        /// @brief A pure virtual function that must be implemented by derived classes to register states and their callbacks with the state machine.
        virtual void registerStates() = 0;

    public:
        /// @brief Constructs a Subsystem with a reference to the robot's time and an initial state.
        /// @details Initializes the subsystem and its internal state machine with the provided initial state.
        /// @param robotTime A reference to the robot's time.
        /// @param initialState The initial state to set for the state machine.
        Subsystem(uint32_t& robotTime, StateEnum initialState) : stateMachine({}, initialState, robotTime), robotTime(robotTime) {}

        /// @brief The state machine that manages the states and transitions for the subsystem.
        StateMachine<StateEnum> stateMachine;

        /// @brief A pure virtual function that must be implemented by derived classes to define the periodic behavior of the subsystem.
        virtual void periodic() = 0;
    };
}
