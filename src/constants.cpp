#include "constants.hpp"

namespace Ports {
    // Drivetrain motor ports
    const int FRONT_LEFT_TOP = 1;
    const int FRONT_LEFT_BOTTOM = 2;

    const int FRONT_RIGHT_TOP = 3;
    const int FRONT_RIGHT_BOTTOM = 4;

    const int BACK_LEFT_TOP = 5;
    const int BACK_LEFT_BOTTOM = 6;
    
    const int BACK_RIGHT_TOP = 7;
    const int BACK_RIGHT_BOTTOM = 8;

    // IMU port
    const int IMU = 5;

    // Subsystem A ports 
}

namespace DriveConstants 
{
    // NON-CONFIGURABLE CONSTANTS (const)
    const double MAX_SPEED = 100.0;
    const double MAX_ACCELERATION = 50.0;
    const double MAX_ANGULAR_SPEED = 10.0;
    const double MAX_ANGULAR_ACCELERATION = 5.0;
    const double WHEEL_DIAMETER = 4.0;
    const double GEAR_RATIO = 1.0;

    // CONFIGURABLE CONSTANTS (no additional keywords needed)
}