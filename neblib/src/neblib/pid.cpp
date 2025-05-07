#include "neblib/pid.hpp"

neblib::PID::Gains::Gains(double kP, double kI, double kD, double windupTolerance, bool resetIntegralOnCross) : kP(kP), kI(kI), kD(kD), windupTolerance(windupTolerance), resetIntegralOnCross(resetIntegralOnCross) {};

neblib::PID::SettleTimeExitConditions::SettleTimeExitConditions(double tolerance, int timeStep, int settleTime) : tolerance(tolerance), timeStep(timeStep), settleTime(settleTime), timeSettled(0) {};

bool neblib::PID::SettleTimeExitConditions::isSettled(double error, double _)
{
    if (std::abs(error) <= tolerance)
        timeSettled += timeStep;
    else
        timeSettled = 0;

    return (timeSettled >= settleTime);
}

neblib::PID::DerivativeExitConditions::DerivativeExitConditions(double errorTolerance, double derivativeTolerance) : errorTolerance(errorTolerance), derivativeTolerance(derivativeTolerance) {};

bool neblib::PID::DerivativeExitConditions::isSettled(double error, double derivative)
{
    return error <= errorTolerance && derivative <= derivativeTolerance;
}

neblib::PID::ExitCondition::ExitCondition(const SettleTimeExitConditions &settleTimeExitConditions) : type(neblib::PID::ExitCondition::ExitConditionType::SETTLE_TIME), settleTimeExitConditions(settleTimeExitConditions) {};

neblib::PID::ExitCondition::ExitCondition(const DerivativeExitConditions &derivativeExitConditions) : type(neblib::PID::ExitCondition::ExitConditionType::DERIVATIVE), derivativeExitConditions(derivativeExitConditions) {};

neblib::PID::ExitCondition::ExitCondition(SettleTimeExitConditions &&settleTimeExitConditions) : type(neblib::PID::ExitCondition::ExitConditionType::SETTLE_TIME), settleTimeExitConditions(settleTimeExitConditions) {};

neblib::PID::ExitCondition::ExitCondition(DerivativeExitConditions &&derivativeExitConditions) : type(neblib::PID::ExitCondition::ExitConditionType::DERIVATIVE), derivativeExitConditions(derivativeExitConditions) {};

bool neblib::PID::ExitCondition::isSettled(double error, double derivative)
{
    switch (type)
    {
    case SETTLE_TIME:
        return settleTimeExitConditions.isSettled(error);
    case DERIVATIVE:
        return derivativeExitConditions.isSettled(error, derivative);
    }
    return false;
}

neblib::PID::PID(const Gains &gains, const SettleTimeExitConditions &settleTimeExitConditions) : gains(gains), exitCondition(settleTimeExitConditions), previousError(0), integral(0), settled(false), initialRun(false) {};

neblib::PID::PID(const Gains &gains, const DerivativeExitConditions &derivativeExitConditions) : gains(gains), exitCondition(derivativeExitConditions), previousError(0), integral(0), settled(false), initialRun(false) {};

neblib::PID::PID(Gains &&gains, SettleTimeExitConditions &&settleTimeExitConditions) : gains(gains), exitCondition(settleTimeExitConditions), previousError(0), integral(0), settled(false), initialRun(false) {};

neblib::PID::PID(Gains &&gains, DerivativeExitConditions &&derivativeExitConditions) : gains(gains), exitCondition(derivativeExitConditions), previousError(0), integral(0), settled(false), initialRun(false) {};

double neblib::PID::getOutput(double error, double minOutput, double maxOutput)
{
    if (std::abs(error) <= gains.windupTolerance)
        integral += error;
    else
        integral = 0;
    if (neblib::sign(error) != neblib::sign(previousError))
        integral = 0;

    double derivative = (initialRun) ? 0 : error - previousError;

    settled = exitCondition.isSettled(error, derivative);

    double output = gains.kP * error + gains.kI * integral + gains.kD * derivative;

    if (output < minOutput)
        return minOutput;
    else if (output > maxOutput)
        return maxOutput;
    else
        return output;
}

double neblib::PID::getOutput(double error) { return getOutput(error, INFINITY, -INFINITY); }

bool neblib::PID::isSettled() { return settled; }
