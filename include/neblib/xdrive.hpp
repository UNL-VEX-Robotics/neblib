#pragma once
#include <functional>
#include <vector>
#include "vex.h"
#include "neblib/tracker_wheel.hpp"
#include "neblib/odometry.hpp"
#include "neblib/pid.hpp"

namespace neblib
{
    class XDrive
    {
    private:
        vex::motor_group leftFront;
        vex::motor_group rightFront;
        vex::motor_group leftBack;
        vex::motor_group rightBack;

        vex::inertial *imu;
        neblib::Odometry *odometry;

        neblib::PID *linearPID;
        neblib::PID *rotationalPID;

    public:
        /// @brief Constructs an XDrive object using vex motor groups
        /// @param leftFront motor group containing the left front motors
        /// @param rightFront motor group containing the right front motors
        /// @param leftBack motor group containing the left rear motors
        /// @param rightBack motor group containing the right rear motors
        /// @param odometry pointer to an odometry object
        /// @param imu pointer to a vex inertial object
        XDrive(vex::motor_group leftFront, vex::motor_group rightFront, vex::motor_group leftBack, vex::motor_group rightBack, neblib::Odometry *odometry, vex::inertial *imu);

        /// @brief Constructs an XDrive object using vex motors
        /// @param leftFront  left front motor
        /// @param rightFront right front motor
        /// @param leftBack left back motor
        /// @param rightBack right back motor
        /// @param odometry pointer to an odometry object
        /// @param imu pointer to a vex inertial object
        XDrive(vex::motor &leftFront, vex::motor &rightFront, vex::motor &leftBack, vex::motor &rightBack, neblib::Odometry *odometry, vex::inertial *imu);

        /// @brief Sets the driving PID used in autonomous
        /// @param pid pointer to a PID object
        void setLinearPID(neblib::PID *pid);

        /// @brief Sets the turning PID used in autonomous
        /// @param pid pointer to a PID object
        void setRotationalPID(neblib::PID *pid);

        /// @brief Drives the robot in a local frame
        /// @param drive forward and backward drive component
        /// @param strafe side to side drive component
        /// @param turn turning component
        /// @param unit velocity unit
        void driveLocal(float drive, float strafe, float turn, vex::velocityUnits unit = vex::velocityUnits::pct);

        /// @brief Drives the robot in a local frame
        /// @param drive forward and backward drive component
        /// @param strafe side to side drive component
        /// @param turn turning component
        /// @param unit voltage unit
        void driveLocal(float drive, float strafe, float turn, vex::voltageUnits unit);

        /// @brief Drives the robot using a target angle and velocity
        /// @param velocity desired velocity
        /// @param deg desired angle
        /// @param turn turning component
        /// @param unit velocity unit
        void drvieAngle(float velocity, float deg, float turn, vex::velocityUnits unit = vex::velocityUnits::pct);

        /// @brief Drives the robot using a target angle and velocity
        /// @param velocity desired velocity
        /// @param deg desired angle
        /// @param turn turning component
        /// @param unit voltage unit
        void driveAngle(float velocity, float deg, float turn, vex::voltageUnits unit);

        /// @brief Drives the robot in the global frame
        /// @param x x component
        /// @param y y component
        /// @param turn turn component
        /// @param unit velocity unit
        void driveGlobal(float x, float y, float turn, vex::velocityUnits unit = vex::velocityUnits::pct);

        /// @brief Drives the robot in the global frame
        /// @param x x component
        /// @param y y component
        /// @param turn turn component
        /// @param unit voltage unit
        void driveGlobal(float x, float y, float turn, vex::voltageUnits unit);

        /// @brief Stops the drivetrain with the desired stop type
        /// @param stopType vex::brakeType to be used, hold by default
        void stop(vex::brakeType stopType = vex::brakeType::hold);

        /// @brief Turns the robot for a set number of degrees
        /// @param deg desired degrees
        /// @param minOutput minimum PID output
        /// @param maxOutput maximum PID output
        /// @param timeout time before PID stops attempting to run if not settled, seconds
        /// @return time it took the function to run, in seconds
        float turnFor(float deg, float minOutput, float maxOutput, float timeout = infinityf());

        /// @brief Turns the robot for a set number of degrees
        /// @param deg desired degrees
        /// @param timeout time before PID stops attempting to run if not settled, seconds
        /// @return time it took the function to run, in seconds
        float turnFor(float deg, float timeout = infinityf());

        /// @brief Turns the robot to a set heading
        /// @param heading desired heading
        /// @param minOutput minimum PID output
        /// @param maxOutput maximum PID output
        /// @param timeout time before PID stops attempting to run if not settled, seconds
        /// @return time it took the function to run, in seconds
        float turnTo(float heading, float minOutput, float maxOutput, float timeout = infinityf());

        /// @brief Turns the robot to a set heading
        /// @param heading desired heading
        /// @param timeout time before PID stops attempting to run if not settled, seconds
        /// @return time it took the function to run, in seconds
        float turnTo(float deg, float timeout = infinityf());

        float driveTo(float x, float y, float minOutput, float maxOutput, float timeout = infinityf());
        float driveTo(float x, float y, float timeout = infinityf());
        float driveToPose(float x, float y, float heading, float minOutput, float maxOutput, float timeout = infinityf());
        float driveToPose(float x, float y, float heading, float timeout = infinityf());
    };
}