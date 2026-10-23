#pragma once

#include "vex.h"
#include "neblib/state_machine/state_machine.hpp"
#include "neblib/state_machine/subsystem.hpp"

namespace neblib {

    class Superstructure {
        public:   /// PUBLIC ///

        enum SuperstructureState { // TODO RENAME THIS ENUM AND CREATE STATES
            IDLE,
        };

        Superstructure(uint32_t& robotTime);

        StateMachine<SuperstructureState> stateMachine;

        void periodic();

        private:   /// PRIVATE ///

        // Create the subsystems here

        uint32_t& robotTime;

        void registerStates();
    };
}
