#include "vex.h"
#include "robot_container.hpp"

namespace neblib {
    RobotContainer::RobotContainer(uint32_t& robotTime) 
        : superstructure(robotTime), 
          xDrive(), 
          driverController() 
    {
        configureButtonBindings();
    }

    void RobotContainer::periodic() 
    {
        superstructure.periodic();
    }

    void RobotContainer::configureButtonBindings() 
    {
        // Configure the button bindings for the robot here
    }
}