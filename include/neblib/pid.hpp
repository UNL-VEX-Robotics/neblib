#pragma once
#include <cmath>
#include "neblib/util.hpp"

namespace neblib
{   
    /// @brief Class used for PIDs
    class PID
    {
    public:
        /// @brief Struct to contain PID gains
        struct Gains
        {
            double kP;
            double kI;
            double kD;
            double windupRange;
            bool resetWindupOnSignChange;

            /// @brief Constructs a Gains object
            /// @param kP proportional constant
            /// @param kI integral constant
            /// @param kD derivative constant
            /// @param windupRange range where integral is used
            /// @param resetWindupOnSignChange resets integral if sign is changed
            Gains(double kP, double kI, double kD, double windupRange, bool resetWindupOnSignChange = true);

            /// @brief Applies the gains to input and returns a final output
            /// @param error current error
            /// @param integral current integral value
            /// @param derivative current derivative value
            /// @return output
            double applyGains(double error, double integral, double derivative);
        };

        /// @brief Base class for PID Exit Conditions
        class ExitConditions
        {
        public:
            // pure virtual methods that need to be implemented
            virtual ~ExitConditions() = default;
            virtual bool isSettled(double error, double derivative) = 0;
        };

        /// @brief Exit conditions using settle time
        class SettleTimeExitConditions : public ExitConditions
        {
        private:
            double tolerance;
            int settleTime;
            int cycleTime;
            int timeSettled;

        public:
            /// @brief Constructs a SettleTimeExitConditions object
            /// @param tolerance tolerance where settle time is counted
            /// @param settleTime time within tolerance to be settled, in mS
            /// @param cycleTime length of time each loop iteration is, in mS
            SettleTimeExitConditions(double tolerance, int settleTime, int cycleTime);

            /// @brief Determines if the PID is settled
            /// @param error current error
            /// @param _ placeholder
            /// @return true if settled, false otherwise
            bool isSettled(double error, double _) override;
        };

        /// @brief Exit conditions using derivative
        class DerivativeExitConditions : public ExitConditions
        {
        private:
            double errorTolerance;
            double derivativeTolerance;

        public:
            /// @brief Constructs a DerivativeExitConditions object
            /// @param errorTolerance tolerance for error
            /// @param derivativeTolerance tolerance for derivative
            DerivativeExitConditions(double errorTolerance, double derivativeTolerance);

            /// @brief Determines if the PID is settled
            /// @param error current error
            /// @param derivative current derivative
            /// @return true if settled, false otherwise
            bool isSettled(double error, double derivative) override;
        };

    private:
        Gains gains;
        std::shared_ptr<ExitConditions> exitConditions;
        double integral;
        double previousError;
        bool hasPreviousError;
        bool settled;
    public:
        /// @brief Constructs a PID object
        /// @param gains Gains object
        /// @param exitConditions heap allocated exit conditions
        PID(Gains&& gains, std::shared_ptr<ExitConditions> exitConditions);

        /// @brief Constructs a PID object
        /// @param gains Gains object
        /// @param exitConditions heap allocated exit conditions
        PID(Gains& gains, std::shared_ptr<ExitConditions> exitConditions);

        /// @brief Constructs a PID object
        /// @param kP proportional constant
        /// @param kI integral constant
        /// @param kD derivative constant
        /// @param windupRange range where integral is used
        /// @param exitConditions heap allocated exit conditions
        /// @param resetWindupOnSignChange resets integral if sign changes
        PID(double kP, double kI, double kD, double windupRange, std::shared_ptr<ExitConditions> exitConditions, bool resetWindupOnSignChange = true);

        /// @brief Gets the output limited to a range
        /// @param error current error
        /// @param minOutput minimum acceptable output
        /// @param maxOutput maximum acceptable output
        /// @return current output
        double getOutput(double error, double minOutput, double maxOutput);

        /// @brief Gets the output
        /// @param error current error
        /// @return current output
        double getOutput(double error);
        
        /// @brief Determines if the PID is settled
        /// @return true if settled, false otherwise
        bool isSettled();

        /// @brief Resets the PID
        void reset();
    };
}