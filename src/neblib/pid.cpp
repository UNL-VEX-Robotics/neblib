#include "neblib/pid.hpp"

neblib::PID::Gains::Gains(double kP, double kI, double kD, double windupRange, bool resetWindupOnSignChange) : kP(kP), kI(kI), kD(kD), windupRange(windupRange), resetWindupOnSignChange(resetWindupOnSignChange) {}

double neblib::PID::Gains::applyGains(double error, double integral, double derivative)
{
    return kP * error + kI * integral + kD * derivative;
}

neblib::PID::SettleTimeExitConditions::SettleTimeExitConditions(double tolerance, int settleTime, int cycleTime) : tolerance(tolerance), settleTime(settleTime), cycleTime(cycleTime), timeSettled(0)
{
}

bool neblib::PID::SettleTimeExitConditions::isSettled(double error, double _)
{
    (void)_;
    if (fabs(error) <= tolerance)
        timeSettled += cycleTime;
    else
        timeSettled = 0;
    return timeSettled >= settleTime;
}

neblib::PID::DerivativeExitConditions::DerivativeExitConditions(double errorTolerance, double derivativeTolerance) : errorTolerance(errorTolerance), derivativeTolerance(derivativeTolerance)
{
}

bool neblib::PID::DerivativeExitConditions::isSettled(double error, double derivative)
{
    return (fabs(error) <= errorTolerance) && (derivative <= derivativeTolerance);
}

neblib::PID::PID(Gains &&gains, std::shared_ptr<ExitConditions> exitConditions) : gains(std::move(gains)), exitConditions(exitConditions), integral(0.0), previousError(0.0), hasPreviousError(false), settled(false)
{
}

neblib::PID::PID(Gains &gains, std::shared_ptr<ExitConditions> exitConditions) : gains(gains), exitConditions(exitConditions), integral(0.0), previousError(0.0), hasPreviousError(false), settled(false)
{
}

neblib::PID::PID(double kP, double kI, double kD, double windupRange, std::shared_ptr<ExitConditions> exitConditions, bool resetWindupOnSignChange) : gains(kP, kI, kD, windupRange, resetWindupOnSignChange), exitConditions(exitConditions), integral(0.0), previousError(0.0), hasPreviousError(false), settled(false)
{
}

double neblib::PID::getOutput(double error, double minOutput, double maxOutput)
{
    if (fabs(error) <= gains.windupRange)
        integral += error;
    else
        integral = 0;
    if (sign(error) != sign(previousError) && hasPreviousError && gains.resetWindupOnSignChange)
        integral = 0;

    double derivative = hasPreviousError ? error - previousError : 0;
    previousError = error;
    hasPreviousError = true;

    settled = exitConditions->isSettled(error, derivative);

    return neblib::clamp(gains.applyGains(error, derivative, integral), minOutput, maxOutput);
}

double neblib::PID::getOutput(double error)
{
    return this->getOutput(error, -infinity(), infinity());
}

bool neblib::PID::isSettled()
{
    return settled;
}

void neblib::PID::reset()
{
    integral = 0;
    previousError = 0;
    hasPreviousError = false;
    settled = false;
}