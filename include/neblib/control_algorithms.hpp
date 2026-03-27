#pragma once

#include <cmath>
#include "util.hpp"

namespace neblib
{

    /// @brief Abstract class for feedback controllers
    class FeedbackController
    {
    public:
        virtual ~FeedbackController() = default;

        /// @brief Computes the controller output for a given error.
        ///
        /// @param error Current error (setpoint - measured value)
        /// @param minOutput Minimum allowed output (default: negative infinity)
        /// @param maxOutput Maximum allowed output (default: infinity)
        /// @return Controller output after clamping
        virtual double getOutput(
            double error,
            double minOutput = -infinity(),
            double maxOutput = infinity()) = 0;

        /// @brief Returns whether the controller considers itself "settled".
        /// @return true if settled, false otherwise
        virtual bool isSettled() = 0;

        /// @brief Resets the internal state of the controller.
        virtual void reset() = 0;
    };

    /// @brief Proportional-Integral-Derivative (PID) controller with optional slew limiting
    class PID : public FeedbackController
    {
    public:
        /// @brief PID Gains
        ///
        /// kP: proportional gain
        /// kI: integral gain
        /// kD: derivative gain
        /// kS: max output change per iteration (slew rate)
        struct Gains
        {
            double kP;
            double kI;
            double kD;
            double kS;

            Gains(
                double kP,
                double kI,
                double kD,
                double kS = infinity());
        };

        /// @brief Optional behavior settings for the PID
        ///
        /// integralTolerance: Only accumulate error when within this range
        /// resetIntegralOnSignChange: Resets integral when error changes sign
        struct Behaviors
        {
            double integralTolerance;
            bool resetIntegralOnSignChange;

            Behaviors(
                double integralTolerance = infinity(),
                bool resetIntegralOnSignChange = false);
        };

        /// @brief Conditions for considering the PID "settled"
        ///
        /// settleTolerance: Magnitude of error considered "close enough"
        /// settleTime: Time (ms) within tolerance required to be considered "settled"
        /// dtMS: Iteration step (ms)
        struct ExitConditions
        {
            double settleTolerance;
            int settleTime;
            int dtMS;

            ExitConditions(
                double settleTolerance,
                int settleTime,
                int dtMS = 10);
        };

    private:
        // --- Configuration ---
        Gains gains;
        Behaviors behaviors;
        ExitConditions exitConditions;

        // --- State ---
        double integral; //< Accumulated integral
        double previousError; //< Error from the previous iteration
        bool hasPreviousError; //< True if previousError is valid
        double previousOutput; //< Output from the previous iteration, used in slew limiting
        int timeSettled; //< Time (ms) within settle tolerance

    public:
        /// @brief Construct a new PID controller
        ///
        /// @param gains PID gains
        /// @param behaviors Optional behavior settings
        /// @param exitConditions Exit conditions for settling
        PID(
            Gains gains,
            Behaviors behaviors,
            ExitConditions exitConditions);

        /// @brief Computes PID output
        ///
        /// @param error Current error (setpoint - measured value)
        /// @param minOutput Minimum allowed output (default: negative infinity)
        /// @param maxOutput Maximum allowed output (default: infinity)
        /// @return PID output after clamping and applying slew
        double getOutput(
            double error,
            double minOutput = -infinity(),
            double maxOutput = infinity()) override;

        /// @brief Checks if PID has settled according to exit conditions
        /// @return true if error has remained within settleTolerance for
        ///         at least settleTime, false otherwise
        bool isSettled() override;

        /// @brief Resets PID state
        void reset() override;
    };

} // namespace neblib