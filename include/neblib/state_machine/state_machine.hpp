#pragma once

#include "vex.h"
#include <cstdint>
#include <functional>
#include <map>

namespace neblib 
{
    /// @brief A generic state machine implementation that allows for state registration, transitions, and callbacks.
    /// @details The StateMachine class is a template that takes an enum type as its state representation. It provides 
    ///          methods to register states with entry, update, and exit callbacks, manage state transitions, and handle 
    ///          periodic updates.
    /// @tparam StateEnum An enum type representing the states of the state machine. Each state should be unique and defined in the enum.
    template <typename StateEnum>
    class StateMachine 
    {
    private:

        /// @brief A reference to the robot's time, used for timing state transitions and callbacks.
        uint32_t& robotTime;

        /// @brief The start time of the state machine, used for timing state transitions and callbacks.
        uint32_t startTime = 0;

        /// @brief Indicates whether a state change is pending.
        bool pendingStateChange;

        /// @brief The pending state to transition to, if a state change is requested.
        StateEnum pendingState;

        /// @brief The current state of the state machine.
        StateEnum currentState;

        /// @brief The disabled state of the state machine, used to represent a non-active state.
        StateEnum disabledState;

        /// @brief Indicates whether the state machine is stopped.
        bool stopped;

        /// @brief Indicates whether a callback is currently being executed.
        bool executingCallback;

    public:
        /// @brief Struct to hold the entry, update, and exit callbacks for a state.
        /// @details Each callback is a std::function<void()> that can be set to any 
        ///          callable object (function pointer, lambda, functor, etc.) that 
        ///          takes no parameters and returns void.
        struct StateCallbacks 
        {
            std::function<void()> entry = [](){};
            std::function<void()> update = [](){};
            std::function<void()> exit = [](){};

            StateCallbacks(std::function<void()> en, std::function<void()> up, std::function<void()> ex)
                : entry(en), update(up), exit(ex) {}
            StateCallbacks() = default;
        };

        /// @brief Struct to hold a state and its associated callbacks for registration.
        /// @details This struct is used to register a state with the state machine,
        ///          associating it with its entry, update, and exit callbacks.
        template <typename State>
        struct Registration 
        {
            State state;
            StateCallbacks stateCallbacks;

            Registration(State state, StateCallbacks stateCallbacks)
                : state(state), stateCallbacks(stateCallbacks) {}
            Registration() = default;
        };

        /// @brief Constructs a StateMachine with an initial state and a reference to the robot's time.
        /// @details Initializes the state machine with the provided initial state and sets up the internal state.
        ///          Upon creation, the state machine is stopped and will not process any state updates until started.
        /// @param initialState The initial state to set for the state machine, wrapped in a Registration struct that includes its callbacks.
        /// @param robotTime A reference to a uint32_t variable that represents the robot's time, used for timing state transitions and callbacks.
        StateMachine(
            Registration<StateEnum> initialState, 
            uint32_t& robotTime) 
            : robotTime(robotTime) 
        {
            stopped = true;
            pendingStateChange = false;
            executingCallback = false;
            registerState(initialState);
            pendingState = initialState.state;
            currentState = initialState.state;
        }

        /// @brief Constructs a StateMachine with a set of registered states and an initial state.
        /// @details Initializes the state machine with the provided registered states and sets up the internal state.
        ///          Upon creation, the state machine is stopped and will not process any state updates until started.
        /// @param registrations A map of state enums to their corresponding registration structs.
        /// @param initialState The initial state to set for the state machine.
        /// @param robotTime A reference to a uint32_t variable that represents the robot's time, used for timing state transitions and callbacks.
        StateMachine(
            std::map<StateEnum, Registration<StateEnum>> registrations, 
            StateEnum initialState,
            uint32_t& robotTime) 
            : robotTime(robotTime) 
        {
            stopped = true;
            pendingStateChange = false;
            executingCallback = false;
            for (const auto& pair : registrations) 
            {
                registerState(pair.second);
            }
            pendingState = initialState;
            currentState = initialState;
        }

        /// @brief Periodically called to handle state updates and transitions.
        /// @details This method should be called in a loop or timer to allow the state machine 
        ///          to process state updates and handle any pending state transitions. It checks 
        ///          if the state machine is stopped or currently executing a callback, and if not, 
        ///          it processes any pending state changes and executes the update callback for the 
        ///          current state.
        void periodic() 
        {
            if (stopped || executingCallback) return;
            if (pendingStateChange) 
            {
                reset(pendingState);
            }
            if (registrationExists(getCurrentState())) 
            {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.update);
            }
        }

        /// @brief Requests a state change.
        /// @details This method is used to request a change to a different state. If the state 
        ///          machine is not stopped, and the requested state is registered, the state 
        ///          change will be pending.
        /// @param state The state to change to, represented by the StateEnum type.
        /// @return true if the state change is requested successfully, false otherwise.
        bool requestStateChange(StateEnum state) 
        {
            if (stopped || pendingStateChange || !(registrationExists(state))) return false;
            if (state != getCurrentState()) 
            {
                pendingState = state;
                pendingStateChange = true;
            }
            return true;
        }

        /// @brief Starts the state machine.
        /// @details This method starts the state machine, allowing it to process state updates 
        ///          and transitions. If the state machine is already running or is currently executing 
        ///          a callback, this method does nothing. If the current state is registered, it executes 
        ///          the entry callback for that state.
        void start() 
        {
            if (!stopped || executingCallback || !registrationExists(getCurrentState())) return;
            captureTime();
            if (registrationExists(getCurrentState())) 
            {
                stopped = false;
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.entry);
            }
        }

        /// @brief Stops the state machine.
        /// @details This method stops the state machine, preventing it from processing further state updates 
        ///          and transitions. If the state machine is already stopped or is currently executing a callback, 
        ///          this method does nothing. If the current state is registered, it executes the exit callback 
        ///          for that state. It will also clear pending work.
        void stop() 
        {
            if (executingCallback) return;
            if (!stopped && registrationExists(getCurrentState())) 
            {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.exit);
            }
            stopped = true;
            pendingStateChange = false;
        }

        /// @brief Resets the state machine to a specific state.
        /// @details This method resets the state machine to the specified state, executing the exit callback 
        ///          for the current state (if registered) and the entry callback for the new state (if registered).
        /// @param state The state to reset to, represented by the StateEnum type.
        /// @return true if the reset is successful, false otherwise.
        bool reset(StateEnum state) 
        {
            if (!registrationExists(state) || executingCallback) return false;
            pendingStateChange = false;
            if (!stopped) 
            {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.exit);
            }
            setCurrentState(state);
            stopped = false;
            captureTime();
            executeCallback(registrations.find(state)->second.stateCallbacks.entry);
            return true;
        }

        /// @brief Resets the state machine to its current state.
        /// @details This method resets the state machine to its current state, executing the exit callback for 
        ///          the current state (if registered) and the entry callback for the current state (if registered).
        /// @return true if the reset is successful, false otherwise.
        bool reset() 
        {
            // Exit early if within a callback or if the current state is not registered
            if (executingCallback || !registrationExists(getCurrentState())) return false;

            // Clear pendingStateChange for the exit callback 
            pendingStateChange = false;

            if (!stopped) 
            {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.exit);
            }

            stopped = false;
            captureTime();
            executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.entry);
            
            return true;
        }

        /// @brief Registers a new state with the state machine.
        /// @param state The state to register, represented by the StateEnum type.
        /// @param entry The entry callback for the state.
        /// @param update The update callback for the state.
        /// @param exit The exit callback for the state.
        /// @return true if the registration does not exist and the machine is stopped, false otherwise.
        bool registerState(StateEnum state, 
            std::function<void()> entry,
            std::function<void()> update,
            std::function<void()> exit) 
        {
            if (registrationExists(state) || !stopped) return false;
            registrations[state] = Registration<StateEnum>(state, StateCallbacks(entry, update, exit));
            return true;
        }

        /// @brief Registers a new state with the state machine.
        /// @param registration The registration information for the state.
        /// @return true if the registration does not exist and the machine is stopped, false otherwise.
        bool registerState(Registration<StateEnum> registration) 
        {
            if (registrationExists(registration.state) || !stopped) return false;
            registrations[registration.state] = registration;
            return true;
        }

        /// @brief Gets the start time of the state machine.
        /// @return The start time of the state machine.
        uint32_t getStartTime() 
        {
            return startTime;
        }

        /// @brief Sets the start time of the state machine.
        /// @param time The time to set as the start time of the state.
        void setStartTime(uint32_t time) 
        {
            startTime = time;
        }

        /// @brief Gets the pending state of the state machine.
        /// @details The pending state is the state that the state machine is set to transition to, 
        ///          if a state change has been requested. It will return stale requests as well, so
        ///          it is recommended to check if a state change is pending before using this value.
        /// @return The pending state of the state machine.
        StateEnum getPendingState() 
        {
            return pendingState;
        }

        /// @brief Gets the current state of the state machine.
        /// @return The current state of the state machine.
        StateEnum getCurrentState() 
        {
            return currentState;
        }

    private:
        /// @brief A map of registered states and their associated callbacks.
        std::map<StateEnum, Registration<StateEnum>> registrations;

        /// @brief Sets the pending state to the specified state.
        /// @param state The state to set as the pending state.
        /// @return true if the pending state was changed, false otherwise.
        bool setPendingState(StateEnum state) 
        {
            if (state != pendingState) 
            {
                pendingState = state;
                return true;
            }
            return false;
        }

        /// @brief Sets the current state to the specified state.
        /// @param state The state to set as the current state.
        /// @return true if the current state was changed, false otherwise.
        bool setCurrentState(StateEnum state) 
        {
            if (state != currentState) 
            {
                currentState = state;
                return true;
            }
            return false;
        }

        /// @brief Executes the specified callback if it is valid.
        /// @param callback The callback to execute.
        void executeCallback(const std::function<void()>& callback) 
        {
            if (callback) 
            {
                executingCallback = true;
                callback();
                executingCallback = false;
            }
        }

        /// @brief Checks if a registration exists for the specified state.
        /// @param state The state to check for a registration.
        /// @return true if a registration exists, false otherwise.
        bool registrationExists(StateEnum state) 
        {
            return registrations.find(state) != registrations.end();
        }

        /// @brief Captures the current robot time and sets it as the start time for the current state.
        void captureTime() 
        {
            setStartTime(robotTime);
        }
    };
}