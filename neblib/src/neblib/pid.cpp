#include "neblib/pid.hpp"


neblib::PID::Gains::Gains(double kP, double kI, double kD, double windupTolerance): kP(kP), kI(kI), kD(kD), windupTolerance(windupTolerance) {};

neblib::PID::SettleTimeExitConditions::SettleTimeExitConditions(double tolerance, int timeStep, int settleTime): tolerance(tolerance), timeStep(timeStep), settleTime(settleTime), timeSettled(0) {};

bool neblib::PID::SettleTimeExitConditions::isSettled(double error, double _ = 0) {
    if (std::abs(error) <= tolerance) timeSettled += timeStep;
    else timeSettled = 0;

    return (timeSettled >= settleTime);
}