#include "neblib/pid.hpp"

neblib::PID::Gains::Gains(double kP, double kI, double kD, double windupRange, bool resetWindupOnSignChange): kP(kP), kI(kI), kD(kD), windupRange(windupRange), resetWindupOnSignChange(resetWindupOnSignChange) {}

neblib::PID::DerivativeExitConditions::DerivativeExitConditions(double errorTolerance, double derivativeTolerance): errorTolerance(errorTolerance), derivativeTolerance(derivativeTolerance) {}

bool neblib::PID::DerivativeExitConditions::isSettled(double error, double derivative) { return error <= std::abs(errorTolerance) && derivative <= std::abs(derivativeTolerance); }

neblib::PID::SettleTimeExitConditions::SettleTimeExitConditions(double tolerance, int settleTime, int cycleTime): tolerance(tolerance), settleTime(settleTime), timeSettled(0), cycleTime(cycleTime) {}

bool neblib::PID::SettleTimeExitConditions::isSettled(double error)
{
    if (error <= std::abs(tolerance)) timeSettled += cycleTime;
    else timeSettled = 0;

    return timeSettled >= settleTime;
}

neblib::PID::ExitConditions::ExitConditions(const DerivativeExitConditions &derivativeExitConditions): type(neblib::PID::ExitConditions::ExitConditionType::DERIVAIVE), derivativeExitConditions(derivativeExitConditions) {}

neblib::PID::ExitConditions::ExitConditions(DerivativeExitConditions &&derivativeExitConditions): type(neblib::PID::ExitConditions::ExitConditionType::DERIVAIVE), derivativeExitConditions(derivativeExitConditions) {}

neblib::PID::ExitConditions::ExitConditions(const SettleTimeExitConditions &settleTimeExitConditions): type(neblib::PID::ExitConditions::ExitConditionType::SETTLE_TIME), settleTimeExitConditions(settleTimeExitConditions) {}

neblib::PID::ExitConditions::ExitConditions(SettleTimeExitConditions &&settleTimeExitConditions): type(neblib::PID::ExitConditions::ExitConditionType::SETTLE_TIME), settleTimeExitConditions(settleTimeExitConditions) {}

bool neblib::PID::ExitConditions::isSettled(double error, double derivative)
{
    switch (type)
    {
        case DERIVAIVE: return derivativeExitConditions.isSettled(error, derivative);
        case SETTLE_TIME: return settleTimeExitConditions.isSettled(error);
        default: return false;
    }
}

neblib::PID::PID(const Gains &gains, const DerivativeExitConditions &derivativeExitConditions): gains(gains), exitConditions(ExitConditions(derivativeExitConditions)), settled(false), integral(0), hasPreviousError(false) {}

neblib::PID::PID(Gains &&gains, DerivativeExitConditions &&derivativeExitConditions): gains(gains), exitConditions(ExitConditions(derivativeExitConditions)), settled(false), integral(0), hasPreviousError(false) {}

neblib::PID::PID(const Gains &gains, const SettleTimeExitConditions &settleTimeExitConditions): gains(gains), exitConditions(ExitConditions(settleTimeExitConditions)), settled(false), integral(0), hasPreviousError(false) {}

neblib::PID::PID(Gains &&gains, SettleTimeExitConditions &&settleTimeExitConditions): gains(gains), exitConditions(ExitConditions(settleTimeExitConditions)), settled(false), integral(0), hasPreviousError(false) {}

double neblib::PID::getOutput(double error)
{
    if (error <= std::abs(gains.windupRange)) integral += error;
    else integral = 0;

    if (hasPreviousError)
    {
        if (sign(error) != sign(previousError)) integral = 0;
    }

    double derivative;
    if (hasPreviousError) derivative = error - previousError;
    else 
    {
        derivative = 0;
        hasPreviousError = true;
    }
    previousError = error;

    settled = exitConditions.isSettled(error, derivative);

    return gains.kP * error + gains.kI * integral + gains.kD * derivative;
}

double neblib::PID::getOutput(double error, double minOutput, double maxOutput)
{
    double output = getOutput(error);
    if (minOutput > output) return minOutput;
    if (maxOutput < output) return maxOutput;
    return output;
}

bool neblib::PID::isSettled() { return settled; }