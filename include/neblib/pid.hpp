#pragma once
#include <cmath>
#include "neblib/util.hpp"

namespace neblib
{
    /// @brief Class for PIDs
    class PID
    {
    public:
        /// @brief Struct for gains
        struct Gains
        {
            double kP;
            double kI;
            double kD;
            double windupRange;
            bool resetWindupOnSignChange;

            /**
             * @brief Constructor for the PID Gains struct
             *
             * @param kP proportional gain
             * @param kI integral gain
             * @param kD derivative gain
             * @param windupRange range where the integral winds up
             * @param resetWindupOnSignChange determines if the integral gets reset when you pass the target
             */
            Gains(double kP, double kI, double kD, double windupRange, bool resetWindupOnSignChange = true);
        };

        /// @brief Class for exit conditions using derivative and error tolerance
        class DerivativeExitConditions
        {
        private:
            double errorTolerance;
            double derivativeTolerance;

        public:
            /**
             * @brief Constructor for the PID derivative based exit condition
             *
             * @param errorTolerance range that error must be within to exit
             * @param derivativeTolerance range that derivative must be within to exit
             */
            DerivativeExitConditions(double errorTolerance, double derivativeTolerance);

            /**
             * @brief Determines if the PID has met the exit conditions
             *
             * @param error the current error of the PID
             * @param derivative the current derivative of the PID
             *
             * @return true if PID has met the exit conditions, false otherwise
             */
            bool isSettled(double error, double derivative);
        };

        /// @brief Class for exit conditions using error tolerance and settle time
        class SettleTimeExitConditions
        {
        private:
            double tolerance;
            int settleTime;
            int timeSettled;
            int cycleTime;

        public:
            /**
             * @brief Constructor for the PID settle time based exit conditions
             *
             * @param tolerance the error tolerance to meet the exit conditions
             * @param settleTime the length of time (milliseconds) that the error must be within the tolerance
             * @param cycleTime the length of time (milliseconds) that the loop takes
             */
            SettleTimeExitConditions(double tolerance, int settleTime, int cycleTime);

            /**
             * @brief Determines if the PID exit conditions have been met
             *
             * @param error the current PID error
             *
             * @return true if the exit conditions have been met, false otherwise
             */
            bool isSettled(double error);
        };

    public:
        /// @brief Wrapper class to contain derivative and settle time exit conditions
        class ExitConditions
        {
        private:
            enum ExitConditionType
            {
                DERIVAIVE,
                SETTLE_TIME
            };

            ExitConditionType type;
            union
            {
                DerivativeExitConditions derivativeExitConditions;
                SettleTimeExitConditions settleTimeExitConditions;
            };

        public:
            /**
             * @brief Constructor for ExitConditions using DerivativeExitConditions passed by reference
             */
            ExitConditions(const DerivativeExitConditions &derivativeExitConditions);
            /**
             * @brief Constructor for ExitConditions using DerivativeExitConditions using move semantics
             */
            ExitConditions(DerivativeExitConditions &&derivativeExitConditions);

            /**
             * @brief Constructor for ExitConditions using SettleTimeExitConditions passed by reference
             */
            ExitConditions(const SettleTimeExitConditions &settleTimeExitConditions);
            /**
             * @brief Constructor for ExitConditions using SettleTimeExitConditions using move semantics
             */
            ExitConditions(SettleTimeExitConditions &&settleTimeExitConditions);

            /**
             * @brief Determines if the PID has settled
             *
             * @param error the current error of the PID
             * @param derivative the current derivative of the PID
             *
             * @return true if PID has met exit conditions, false otherwise
             */
            bool isSettled(double error, double derivative);
        };

        Gains gains;
        ExitConditions exitConditions;
        bool settled;
        double integral;
        double previousError;
        bool hasPreviousError;

    public:
        /**
         * @brief Constructor for PID using derivative based exit conditions passed by reference
         *
         * @param gains Gains struct
         * @param derivativeExitConditions derivative based exit conditions
         */
        PID(const Gains &gains, const DerivativeExitConditions &derivativeExitConditions);
        /**
         * @brief Constructor for PID using derivative based exit conditions using move semantics
         *
         * @param gains Gains struct
         * @param derivativeExitConditions derivative based exit conditions
         */
        PID(Gains &&gains, DerivativeExitConditions &&derivativeExitConditions);

        /**
         * @brief Constructor for PID using settle time based exit conditions passed by reference
         *
         * @param gains Gains struct
         * @param settleTimeExitConditions settle time based exit conditions
         */
        PID(const Gains &gains, const SettleTimeExitConditions &settleTimeExitConditions);
        /**
         * @brief Constructor for PID using settle time based exit conditions using move semantics
         *
         * @param gains Gains struct
         * @param settleTimeExitConditions settle time based exit conditions
         */
        PID(Gains &&gains, SettleTimeExitConditions &&settleTimeExitConditions);

        /**
         * @brief Returns the output of the PID
         *
         * @param error the current error or distance to the target
         *
         * @return the PID output
         */
        double getOutput(double error);
        /**
         * @brief Returns the output of the PID
         *
         * @param error the current error or distance to the target
         * @param minOutput the minimum acceptable output value
         * @param maxOutput the maximum acceptable output value
         *
         * @return the PID output
         */
        double getOutput(double error, double minOutput, double maxOutput);

        /**
         * @brief Determines if the PID is settled
         *
         * @return true if PID is settled, false otherwise
         */
        bool isSettled();

        /// @brief Resets the PID object
        void reset();
    };

}