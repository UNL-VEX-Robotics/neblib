#include "neblib/state_machine/subsystem.hpp"
#include "neblib/state_machine/state_machine.hpp"
#include <cstdint>

enum SubsystemState { // TODO RENAME THIS ENUM AND CREATE STATES
    IDLE,
};

using namespace neblib;

class MySubsystem : public Subsystem<SubsystemState> { // TODO RENAME THIS CLASS
public:
    MySubsystem(uint32_t& robotTime) : Subsystem<SubsystemState>(robotTime, SubsystemState::IDLE) {
        // Create the motors and other devices for the subsystem here

        // Register the states for the subsystem 
        registerStates();

        stateMachine.start(); // Start the state machine
    }

    void periodic() override {
        // Implement the periodic behavior of the subsystem here
        stateMachine.periodic();
    }

private:
    void registerStates() override {
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
