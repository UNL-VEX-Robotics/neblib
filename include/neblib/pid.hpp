#pragma once
#include <cmath>
#include "neblib/util.hpp"

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
            double windupRange;
            bool resetWindupOnSignChange;

            Gains(double kP, double kI, double kD, double windupRange, bool resetWindupOnSignChange = true);
            double applyGains(double error, double integral, double derivative);
        };

        class ExitConditions
        {
        public:
            virtual ~ExitConditions() = default;
            virtual bool isSettled(double error, double derivative) = 0;
        };

        class SettleTimeExitConditions : public ExitConditions
        {
        private:
            double tolerance;
            int settleTime;
            int cycleTime;
            int timeSettled;

        public:
            SettleTimeExitConditions(double tolerance, int settleTime, int cycleTime);

            bool isSettled(double error, double _) override;
        };

        class DerivativeExitConditions : public ExitConditions
        {
        private:
            double errorTolerance;
            double derivativeTolerance;

        public:
            DerivativeExitConditions(double errorTolerance, double derivativeTolerance);

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
        PID(Gains&& gains, std::shared_ptr<ExitConditions> exitConditions);
        PID(Gains& gains, std::shared_ptr<ExitConditions> exitConditions);
        PID(double kP, double kI, double kD, double windupRange, std::shared_ptr<ExitConditions> exitConditions, bool resetWindupOnSignChange = true);

        double getOutput(double error, double minOutput, double maxOutput);
        double getOutput(double error);
        
        bool isSettled();

        void reset();
    };
}