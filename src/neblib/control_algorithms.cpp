#include "control_algorithms.hpp"

neblib::PID::Gains::Gains(
    double kP,
    double kI,
    double kD,
    double kS)
    : kP(kP),
      kI(kI),
      kD(kD),
      kS(kS)
{
}

neblib::PID::Behaviors::Behaviors(
    double integralTolerance,
    bool resetIntegralOnSignChange)
    : integralTolerance(integralTolerance),
      resetIntegralOnSignChange(resetIntegralOnSignChange)
{
}

neblib::PID::ExitConditions::ExitConditions(
    double settleTolerance,
    double settleTime,
    int dtMS)
    : settleTolerance(settleTolerance),
      settleTime(settleTime),
      dtMS(dtMS)
{
}

neblib::PID::PID(
    Gains gains,
    Behaviors behaviors,
    ExitConditions exitConditions)
    : gains(gains),
      behaviors(behaviors),
      exitConditions(exitConditions),
      integral(0.0),
      previousError(0.0),
      hasPreviousError(false),
      previousOutput(0.0),
      timeSettled(0)
{
}

double neblib::PID::getOutput(
    double error,
    double minOutput,
    double maxOutput)
{
    // Calculate Integral
    if (std::abs(error) <= behaviors.integralTolerance)
        integral += error;
    if (behaviors.resetIntegralOnSignChange && hasPreviousError && neblib::sign(error) != neblib::sign(previousError))
        integral = 0.0;

    // Calculate Derivative
    const double derivative = (hasPreviousError) ? error - previousError : error;

    // Calculate Output
    double output = neblib::clamp(gains.kP * error + gains.kI * integral + gains.kD * derivative, minOutput, maxOutput);

    // Apply Slew
    output = neblib::clamp(output, previousOutput - gains.kS, previousOutput + gains.kS);

    // Update State
    previousError = error;
    hasPreviousError = true;

    previousOutput = output;

    if (std::abs(error) < exitConditions.settleTolerance)
        timeSettled += exitConditions.dtMS;
    else
        timeSettled = 0;

    // Return Output
    return output;
}

bool neblib::PID::isSettled()
{
    return timeSettled >= exitConditions.settleTime;
}

void neblib::PID::reset()
{
    integral = 0.0;
    previousError = 0.0;
    hasPreviousError = false;
    previousOutput = 0.0;
    timeSettled = 0;
}
