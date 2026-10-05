#pragma once

#include "vex.h"
#include <cstdint>
#include <functional>
#include <unordered_map>

namespace neblib {
    template <typename StateEnum>
    class StateMachine {
        private:
        struct StateCallbacks {
            std::function<void()> entry;
            std::function<void()> update;
            std::function<void()> exit;

            StateCallbacks(std::function<void()> en, std::function<void()> up, std::function<void()> ex)
                : entry(en), update(up), exit(ex) {}
            StateCallbacks() = default;
        };

        template <typename State>
        struct Registration {
            State state;
            StateCallbacks stateCallbacks;
        };

        public:

        StateMachine(StateEnum initialState, uint32_t time) {
            startTime = time;
            pendingState = initialState;
            currentState = initialState;
            stopped = false;
        }

        uint32_t getStartTime();
        void setStartTime();

        StateEnum getPendingState() {
            return pendingState;
        }

        bool setPendingState(StateEnum state) {
            if (state != pendingState) {
                pendingState = state;
                return true;
            }
            return false;
        }

        StateEnum getCurrentState() {
            return currentState;
        }

        bool setCurrentState(StateEnum state) {
            if (state != currentState) {
                currentState = state;
                return true;
            }
            return false;
        }

        std::unordered_map<StateEnum, Registration<StateEnum>> registrations;

        void registerState(StateEnum state, 
            std::function<void()> entry,
            std::function<void()> update,
            std::function<void()> exit) {
            registrations[state] = Registration<StateEnum>(StateCallbacks(entry, update, exit));
        }

        void update() {
            if (pendingState != currentState) {
                reset(pendingState, startTime);
            }
            if (!stopped) {
                registrations[getCurrentState()].stateCallbacks.update();
            }
        }

        void reset(StateEnum state, uint32_t time) {
            if (!stopped) {
                registrations[getCurrentState()].stateCallbacks.exit();
            }
            setCurrentState(state);
            registrations[getCurrentState()].stateCallbacks.entry();
        }

        void stop() {
            if (!stopped) {
                registrations[getCurrentState()].stateCallbacks.exit();
            }
            stopped = true;
        }

        void start() {
            stopped = false;
            registrations[getCurrentState()].stateCallbacks.entry();
        }

        private:

        uint32_t startTime;

        bool pendingStateChange;
        StateEnum pendingState;
        StateEnum currentState;
        StateEnum disabledState;

        bool stopped;
    };
}