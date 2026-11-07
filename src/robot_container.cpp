#include "vex.h"
#include "robot_container.hpp"
#include "constants.hpp"

using namespace vex;

namespace neblib {

    // Motors for the drivetrain, initialized with the ports defined in constants.hpp
    motor frontLeftTop(Ports::FRONT_LEFT_TOP);
    motor frontLeftBottom(Ports::FRONT_LEFT_BOTTOM);

    motor frontRightTop(Ports::FRONT_RIGHT_TOP);
    motor frontRightBottom(Ports::FRONT_RIGHT_BOTTOM);

    motor backLeftTop(Ports::BACK_LEFT_TOP);
    motor backLeftBottom(Ports::BACK_LEFT_BOTTOM);

    motor backRightTop(Ports::BACK_RIGHT_TOP);
    motor backRightBottom(Ports::BACK_RIGHT_BOTTOM);

    // Motor groups for the drivetrain, grouping the top and bottom motors for each side
    motor_group leftFront(frontLeftTop, frontLeftBottom);
    motor_group rightFront(frontRightTop, frontRightBottom);
    motor_group leftBack(backLeftTop, backLeftBottom);
    motor_group rightBack(backRightTop, backRightBottom);

    // IMU sensor for the drivetrain, initialized with the port defined in constants.hpp
    inertial imu(Ports::IMU);
    
    // Constructing the RobotContainer with references to the robot's time and initializing subsystems
    RobotContainer::RobotContainer(uint32_t& robotTime) 
        : superstructure(robotTime), 
          xDrive(leftFront, 
                 rightFront, 
                 leftBack, 
                 rightBack, 
                 imu,
                 nullptr),
          driver(vex::controllerType::primary),
          partner(vex::controllerType::partner)
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