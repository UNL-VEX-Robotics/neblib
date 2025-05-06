#pragma once
#include <cmath>

namespace neblib {



class PID {
public:
    struct Gains {
        double kP;
        double kI;
        double kD;
        double windupTolerance;

        /// @brief Constructor for PID Gains struct
        /// @param kP proportional gain
        /// @param kI integral gain
        /// @param kD derivative gain
        /// @param windupTolerance range where integral can wind up
        Gains(double kP, double kI, double kD, double windupTolerance);
    };

    class SettleTimeExitConditions {
    private:

        double tolerance;
        int timeStep;

        int settleTime;
        int timeSettled;
    public:

        /// @brief Constructor for PID exit conditions based on settle time
        /// @param tolerance range error must be within to be acceptable
        /// @param timeStep time step of the PID loop, in milliseconds
        /// @param settleTime amount of time error must be within tolerance, in milliseconds
        SettleTimeExitConditions(double tolerance, int timeStep, int settleTime);

        /// @brief Updates the exit conditions and determines if the PID is settled, run once per loop iteration
        /// @return true if settled, false otherwise
        bool isSettled(double error, double _ = 0);
    };

    class DerivativeExitConditions {
    private:

        double errorTolerance;
        double derivativeTolerance;
    public:

        /// @brief Constructor for PID exit conditions based on PID derivative
        /// @param errorTolerance range error must be within to be acceptable
        /// @param derivativeTolerance range derivative must be within to be acceptable
        DerivativeExitConditions(double errorTolerance, double derivativeTolerance);

        /// @brief Updates the exit conditions and determines if the PID is settled, run once per loop iteration
        /// @return true if settled, false otherwise
        bool isSettled(double error, double derivative);
    };

private:

    enum ExitConditionType {
        SETTLE_TIME,
        DERIVATIVE
    };

    class ExitCondition {
        ExitConditionType type;
        union {
            SettleTimeExitConditions settleTimeExitConditions;
            DerivativeExitConditions derivativeExitConditions;
        };

        bool isSettled(double error, double derivatrive);
    };

public:

};

};