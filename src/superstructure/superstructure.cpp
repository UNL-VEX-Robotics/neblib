#include "superstructure/superstructure.hpp"
#include "neblib/state_machine/state_machine.hpp"

namespace neblib {
    
    Superstructure::Superstructure(uint32_t& robotTime) 
        : stateMachine({}, SuperstructureState::IDLE, robotTime), 
        robotTime(robotTime) 
    {
        // Create the subsystems here

        // Register the states for the superstructure 
        registerStates();

        // Start the state machine
        stateMachine.reset();
    }

    void Superstructure::periodic() {
        // Implement the periodic behavior of the superstructure here
        stateMachine.periodic();
    }

    void Superstructure::registerStates() {
        // Here is where you will register the states for the superstructure. 
        // You can add more states and their corresponding callbacks as needed.

        stateMachine.registerState( 
            /// IDLE ///
            StateMachine<SuperstructureState>::Registration<SuperstructureState>(
                SuperstructureState::IDLE, 
                StateMachine<SuperstructureState>::StateCallbacks([](){}, [](){}, [](){})
            )
        );
    }
}
