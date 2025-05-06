#pragma once
#include <cmath>

#include "neblib/utility.hpp"

namespace neblib
{

    class PID
    {
    public:
        struct Gains
        {
            double kP;
            double kI;
            double kD;
            double windupTolerance;
            bool resetIntegralOnCross;

            /// @brief Constructor for PID Gains struct
            /// @param kP proportional gain
            /// @param kI integral gain
            /// @param kD derivative gain
            /// @param windupTolerance range where integral can wind up
            Gains(double kP, double kI, double kD, double windupTolerance, bool resetIntegralOnCross = false);
        };

        class SettleTimeExitConditions
        {
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

        class DerivativeExitConditions
        {
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
        enum ExitConditionType
        {
            SETTLE_TIME,
            DERIVATIVE
        };

        class ExitCondition
        {

        private:
            ExitConditionType type;
            union
            {
                SettleTimeExitConditions settleTimeExitConditions;
                DerivativeExitConditions derivativeExitConditions;
            };

        public:
            /// @brief Constructor for ExitConditions class
            /// @param settleTimeExitConditions exit conditions based on settle time
            ExitCondition(const SettleTimeExitConditions &settleTimeExitConditions);
            /// @brief Constructor for ExitConditions class
            /// @param derivativeExitConditions exit conditions based on derivative
            ExitCondition(const DerivativeExitConditions &derivativeExitConditions);

            /// @brief Constructor for ExitConditions class
            /// @param settleTimeExitConditions exit conditions based on settle time
            ExitCondition(SettleTimeExitConditions &&settleTimeExitConditions);
            /// @brief Constructor for ExitConditions class
            /// @param derivativeExitConditions exit conditions based on derivative
            ExitCondition(DerivativeExitConditions &&derivativeExitConditions);

            /// @brief Determines if the PID is settled
            /// @param error current error
            /// @param derivatrive current derivative
            /// @return true if settled, false otherwise
            bool isSettled(double error, double derivatrive);
        };

        Gains gains;
        ExitCondition exitCondition;

        double previousError;
        double integral;

        bool settled;
        bool initialRun;

    public:
        /// @brief Constructor for PID class
        /// @param gains PID gains
        /// @param settleTimeExitConditions exit conditions based on settle time
        PID(const Gains &gains, const SettleTimeExitConditions &settleTimeExitConditions);
        /// @brief Constructor for PID class
        /// @param gains PID gains
        /// @param derivativeExitConditions exit conditions based on derivative
        PID(const Gains &gains, const DerivativeExitConditions &derivativeExitConditions);

        /// @brief Constructor for PID class
        /// @param gains PID gains
        /// @param settleTimeExitConditions exit conditions based on settle time
        PID(Gains &&gains, SettleTimeExitConditions &&settleTimeExitConditions);
        /// @brief Constructor for PID class
        /// @param gains PID gains
        /// @param derivativeExitConditions exit conditions based on derivative
        PID(Gains &&gains, DerivativeExitConditions &&derivativeExitConditions);

        /// @brief Determines appropriate PID output
        /// @param error error or distance from target
        /// @param minOutput minimum acceptable output
        /// @param maxOutput maximum acceptable output
        /// @return PID output
        double getOutput(double error, double minOutput, double maxOutput);
        /// @brief Determines appropriate PID output
        /// @param error error or distance from target
        /// @return PID output
        double getOutput(double error);

        /// @brief Determines if PID is settled
        /// @return true if settled, false otherwise
        bool isSettled();
    };

};