#include "neblib/state_machine/subsystem.hpp"
#include "neblib/state_machine/state_machine.hpp"
#include <cstdint>

enum SubsystemState { // TODO RENAME THIS ENUM AND CREATE STATES
    IDLE,
};

using namespace neblib;

class MySubsystem : Subsystem<SubsystemState> { // TODO RENAME THIS CLASS
public:
    MySubsystem(uint32_t& robotTime) : Subsystem<SubsystemState>(robotTime) {
        // Create the motors and other devices for the subsystem here

        // Register the states for the subsystem 
        registerStates();

        // Set the initial state of the subsystem
        stateMachine.requestStateChange(IDLE);

        // Start the state machine
        stateMachine.reset();
    }

    void periodic() override {
        // Implement the periodic behavior of the subsystem here
        stateMachine.periodic();
    }

private:
    void registerStates() {
        // Here is where you will register the states for the subsystem. 
        // You can add more states and their corresponding callbacks as needed.
        stateMachine.registerState(
            StateMachine<SubsystemState>::Registration<SubsystemState>(
                IDLE, 
                StateMachine<SubsystemState>::StateCallbacks([](){}, [](){}, [](){})
            )
        );
    }
};
