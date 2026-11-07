
// Ports that correspond to the Brain's physical ports, used for initializing motors and sensors
namespace Ports 
{
    // Drivetrain motor ports
    extern const int FRONT_LEFT_TOP;
    extern const int FRONT_LEFT_BOTTOM;

    extern const int FRONT_RIGHT_TOP;
    extern const int FRONT_RIGHT_BOTTOM;
    
    extern const int BACK_LEFT_TOP;
    extern const int BACK_LEFT_BOTTOM;
    
    extern const int BACK_RIGHT_TOP;
    extern const int BACK_RIGHT_BOTTOM;

    // IMU port
    extern const int IMU;
}

namespace DriveConstants 
{
    // NON-CONFIGURABLE CONSTANTS (extern const)

    /// @brief The maximum speed of the robot in meters per second.
    extern const double MAX_SPEED;

    /// @brief The maximum acceleration of the robot in meters per second squared.
    extern const double MAX_ACCELERATION;

    /// @brief The maximum angular speed of the robot in radians per second.
    extern const double MAX_ANGULAR_SPEED;

    /// @brief The maximum angular acceleration of the robot in radians per second squared.
    extern const double MAX_ANGULAR_ACCELERATION;

    /// @brief The wheel diameter of the robot in meters.
    extern const double WHEEL_DIAMETER;

    /// @brief The gear ratio of the robot's drive system.
    extern const double GEAR_RATIO;

    // CONFIGURABLE CONSTANTS (extern)
}