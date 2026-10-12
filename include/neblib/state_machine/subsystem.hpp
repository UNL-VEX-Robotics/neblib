#pragma once

#include "vex.h"
#include "state_machine.hpp"
#include <cstdint>

namespace neblib {
    template <typename StateEnum>
    class Subsystem {
        public:   /// PUBLIC ///

        Subsystem(uint32_t& robotTime) : robotTime(robotTime) {}

        StateMachine<StateEnum> stateMachine;

        virtual void periodic() = 0;

        protected:   /// PROTECTED ///

        uint32_t& robotTime;

        virtual void registerStates() = 0;
    };
}
