#include "neblib/standard_drive.hpp"

neblib::StandardDrive::StandardDrive(vex::motor_group& leftMotorGroup, vex::motor_group& rightMotorGroup, vex::inertial&& inertial, double trackwidth): leftMotorGroup(&leftMotorGroup), rightMotorGroup(&rightMotorGroup), inertial(inertial), trackWidth(trackWidth) {};