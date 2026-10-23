#pragma once

#include "vex.h"
#include <cstdint>
#include <functional>
#include <map>

namespace neblib {
    template <typename StateEnum>
    class StateMachine {
        public:   /// PUBLIC ///

        //
        // Structs and class definitions
        //
        struct StateCallbacks 
        {
            std::function<void()> entry = [](){};
            std::function<void()> update = [](){};
            std::function<void()> exit = [](){};

            StateCallbacks(std::function<void()> en, std::function<void()> up, std::function<void()> ex)
                : entry(en), update(up), exit(ex) {}
            StateCallbacks() = default;
        };

        template <typename State>
        struct Registration 
        {
            State state;
            StateCallbacks stateCallbacks;

            Registration(State state, StateCallbacks stateCallbacks)
                : state(state), stateCallbacks(stateCallbacks) {}
            Registration() = default;
        };

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

        StateMachine(
            std::map<StateEnum, Registration<StateEnum>> registrations, 
            StateEnum initialState,
            uint32_t& robotTime) 
            : robotTime(robotTime) 
        {
            stopped = true;
            pendingStateChange = false;
            executingCallback = false;
            for (const auto& pair : registrations) {
                registerState(pair.second);
            }
            pendingState = initialState;
            currentState = initialState;
        }


        //
        // Primary state machine methods
        //
        void periodic() 
        {
            if (stopped || executingCallback) return;
            if (pendingStateChange) {
                reset(pendingState);
            }
            if (registrationExists(getCurrentState())) {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.update);
            }
        }

        bool requestStateChange(StateEnum state) 
        {
            if (stopped || pendingStateChange || !(registrationExists(state))) return false;
            if (state != getCurrentState()) {
                pendingState = state;
                pendingStateChange = true;
            }
            return true;
        }

        void start() 
        {
            if (!stopped || executingCallback || !registrationExists(getCurrentState())) return;
            captureTime();
            if (registrationExists(getCurrentState())) {
                stopped = false;
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.entry);
            }
        }

        void stop() 
        {
            if (executingCallback) return;
            if (!stopped && registrationExists(getCurrentState())) {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.exit);
            }
            stopped = true;
            pendingStateChange = false;
        }

        bool reset(StateEnum state) 
        {
            if (!registrationExists(state) || executingCallback) return false;
            pendingStateChange = false;
            if (!stopped) {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.exit);
            }
            setCurrentState(state);
            stopped = false;
            captureTime();
            executeCallback(registrations.find(state)->second.stateCallbacks.entry);
            return true;
        }

        bool reset() 
        {
            // Exit early if within a callback or if the current state is not registered
            if (executingCallback || !registrationExists(getCurrentState())) return false;

            // Clear pendingStateChange for the exit callback 
            pendingStateChange = false;

            if (!stopped) {
                executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.exit);
            }

            stopped = false;
            captureTime();
            executeCallback(registrations.find(getCurrentState())->second.stateCallbacks.entry);
            
            return true;
        }

        bool registerState(StateEnum state, 
            std::function<void()> entry,
            std::function<void()> update,
            std::function<void()> exit) 
        {
            if (registrationExists(state) || !stopped) return false;
            registrations[state] = Registration<StateEnum>(state, StateCallbacks(entry, update, exit));
            return true;
        }

        bool registerState(Registration<StateEnum> registration) 
        {
            if (registrationExists(registration.state) || !stopped) return false;
            registrations[registration.state] = registration;
            return true;
        }

        uint32_t getStartTime() 
        {
            return startTime;
        }

        void setStartTime(uint32_t time) 
        {
            startTime = time;
        }

        StateEnum getPendingState() 
        {
            return pendingState;
        }

        StateEnum getCurrentState() 
        {
            return currentState;
        }


        private:     /// PRIVATE ///

        // Timing
        uint32_t& robotTime;
        uint32_t startTime = 0;

        // State handling
        bool pendingStateChange;
        StateEnum pendingState;
        StateEnum currentState;
        StateEnum disabledState;

        // Flags
        bool stopped;
        bool executingCallback;

        std::map<StateEnum, Registration<StateEnum>> registrations;

        bool setPendingState(StateEnum state) 
        {
            if (state != pendingState) {
                pendingState = state;
                return true;
            }
            return false;
        }

        bool setCurrentState(StateEnum state) 
        {
            if (state != currentState) {
                currentState = state;
                return true;
            }
            return false;
        }

        void executeCallback(const std::function<void()>& callback) 
        {
            if (callback) {
                executingCallback = true;
                callback();
                executingCallback = false;
            }
        }

        bool registrationExists(StateEnum state) 
        {
            return registrations.find(state) != registrations.end();
        }

        void captureTime() 
        {
            setStartTime(robotTime);
        }
    };
}