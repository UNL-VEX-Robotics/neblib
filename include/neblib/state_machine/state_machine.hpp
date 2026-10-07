#pragma once

#include "vex.h"
#include <cstdint>
#include <functional>
#include <unordered_map>

namespace neblib {
    template <typename StateEnum>
    class StateMachine {
        public:
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

            Registration(State state, StateCallbacks stateCallbacks)
                : state(state), stateCallbacks(stateCallbacks) {}
            Registration() = default;
        };

        StateMachine(Registration<StateEnum> initialState, uint32_t time) {
            startTime = time;
            registerState(initialState);
            pendingState = initialState.state;
            currentState = initialState.state;
            pendingStateChange = false;
            start();
        }

        void periodic() {
            if (stopped) return;
            if (pendingStateChange) {
                reset(pendingState);
                return;
            }
            update();
        }

        void requestStateChange(StateEnum state) {
            if (pendingStateChange || !registrations.contains(state)) return;
            if (state != getCurrentState()) {
                pendingState = state;
                pendingStateChange = true;
            }
            return;
        }

        void start() {
            stopped = false;
            registrations[getCurrentState()].stateCallbacks.entry();
        }

        std::unordered_map<StateEnum, Registration<StateEnum>> registrations;

        void registerState(StateEnum state, 
            std::function<void()> entry,
            std::function<void()> update,
            std::function<void()> exit) {
            registrations[state] = Registration<StateEnum>(state, StateCallbacks(entry, update, exit));
        }

        void registerState(Registration<StateEnum> registration) {
            registrations[registration.state] = registration;
        }

        uint32_t getStartTime();
        void setStartTime();

        StateEnum getPendingState() {
            return pendingState;
        }

        StateEnum getCurrentState() {
            return currentState;
        }


        private:

        uint32_t startTime;

        bool pendingStateChange;
        StateEnum pendingState;
        StateEnum currentState;
        StateEnum disabledState;

        bool stopped;

        bool setPendingState(StateEnum state) {
            if (state != pendingState) {
                pendingState = state;
                return true;
            }
            return false;
        }

        bool setCurrentState(StateEnum state) {
            if (state != currentState) {
                currentState = state;
                return true;
            }
            return false;
        }

        void update() {
            registrations[getCurrentState()].stateCallbacks.update();
        }

        void reset(StateEnum state) {
            if (!stopped) {
                registrations[getCurrentState()].stateCallbacks.exit();
            }
            setCurrentState(state);
            pendingStateChange = false;
            registrations[getCurrentState()].stateCallbacks.entry();
        }

        void stop() {
            if (!stopped) {
                registrations[getCurrentState()].stateCallbacks.exit();
            }
            stopped = true;
        }

        void captureTime(); // TODO make this able to capture the current robot time
    };
}