# Control Algorithms

The control-algorithms module provides a general feedback-controller interface and a configurable proportional-integral-derivative (PID) controller. The PID implementation supports integral management, output limiting, slew limiting, and time-based settling detection.

Include the header before using these types:

```cpp
#include "neblib/control_algorithms.hpp"
```

All types on this page are in the `neblib` namespace.

## Feedback-Control Basics

A feedback controller repeatedly compares a target with a measured value and calculates a corrective output:

```cpp
double error = target - measurement;
double output = controller.getOutput(error, minOutput, maxOutput);
```

The meaning of the values depends on the system being controlled. For example, the error might be inches, degrees, or RPM, while the output might be volts or percent power. Gains must be tuned for the units used by the calling code.

## `FeedbackController`

```cpp
class FeedbackController
{
public:
    virtual ~FeedbackController() = default;

    virtual double getOutput(
        double error,
        double minOutput = -infinity(),
        double maxOutput = infinity()) = 0;

    virtual bool isSettled() = 0;
    virtual void reset() = 0;
};
```

`FeedbackController` is the common interface for feedback controllers. Code such as a drive class can use a `FeedbackController` pointer without depending on a particular control algorithm.

The class is abstract and cannot be constructed directly.

### `getOutput`

Calculates the next controller output from the current error.

**Parameters**

- `error` - The current target error, normally calculated as `target - measurement`.
- `minOutput` - The requested lower output limit. It defaults to negative infinity.
- `maxOutput` - The requested upper output limit. It defaults to positive infinity.

**Returns:** The controller's next output.

### `isSettled`

Returns `true` when the controller's exit conditions have been satisfied. The exact definition of settled depends on the implementation.

### `reset`

Clears the controller's stored state. Call this before starting a new movement or control operation so previous errors and outputs do not affect the new operation.

## `PID`

`PID` combines three error terms:

- **Proportional** responds to the current error.
- **Integral** responds to accumulated error.
- **Derivative** responds to the change in error.

The controller calculates an unclamped output conceptually as:

```text
output = kP * error + kI * integral + kD * derivative
```

neblib then applies the requested output limits followed by the configured slew limit.

## PID Configuration

### `PID::Gains`

```cpp
struct Gains
{
    double kP;
    double kI;
    double kD;
    double kS;

    Gains(double kP, double kI, double kD, double kS = infinity());
};
```

Defines the controller gains and slew limit.

**Parameters**

- `kP` - Proportional gain applied to the current error.
- `kI` - Integral gain applied to the accumulated error.
- `kD` - Derivative gain applied to the change in error.
- `kS` - Maximum amount the output may change on each call to `getOutput`. It defaults to infinity, which disables slew limiting.

```cpp
neblib::PID::Gains gains(
    0.8,  // kP
    0.01, // kI
    0.2,  // kD
    0.5); // kS
```

`kS` is a change per iteration, not a change per second. Its real-world rate therefore depends on how frequently `getOutput` is called.

### `PID::Behaviors`

```cpp
struct Behaviors
{
    double integralTolerance;
    bool resetIntegralOnSignChange;

    Behaviors(
        double integralTolerance = infinity(),
        bool resetIntegralOnSignChange = false);
};
```

Controls when the integral accumulates and when it resets.

**Parameters**

- `integralTolerance` - The largest absolute error for which the integral is allowed to accumulate. It defaults to infinity, so all errors accumulate.
- `resetIntegralOnSignChange` - When `true`, resets the integral when the error changes sign.

```cpp
neblib::PID::Behaviors behaviors(
    5.0, // Accumulate only while |error| <= 5.0
    true);
```

When the error is outside `integralTolerance`, the existing integral is retained but no new error is added. It is not automatically cleared. A sign change includes a transition between zero and a positive or negative value because `neblib::sign(0)` returns `0`.

Integral tolerance can reduce integral windup while the system is far from its target. Set `kI` to `0.0` when integral control is not needed.

### `PID::ExitConditions`

```cpp
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
```

Defines when `isSettled` becomes `true`.

**Parameters**

- `settleTolerance` - The absolute-error boundary considered close enough to the target.
- `settleTime` - The number of consecutive milliseconds the error must remain within the tolerance.
- `dtMS` - The expected number of milliseconds between calls to `getOutput`. It defaults to `10`.

```cpp
neblib::PID::ExitConditions exitConditions(
    1.0, // |error| must be less than 1.0
    250, // Remain there for 250 ms
    10); // getOutput is called every 10 ms
```

The tolerance comparison is strict: an error whose absolute value equals `settleTolerance` is not considered within tolerance. Any call with an error outside the tolerance resets the accumulated settling time to zero.

`dtMS` does not measure actual elapsed time. Each call within tolerance adds exactly `dtMS` to the stored settling time. The caller should therefore wait approximately `dtMS` milliseconds between updates.

## Constructor

```cpp
PID(
    Gains gains,
    Behaviors behaviors,
    ExitConditions exitConditions);
```

Creates a PID controller with the supplied configuration and zeroed internal state.

```cpp
neblib::PID controller(
    neblib::PID::Gains(0.8, 0.01, 0.2, 0.5),
    neblib::PID::Behaviors(5.0, true),
    neblib::PID::ExitConditions(1.0, 250, 10));
```

## PID Methods

### `getOutput`

```cpp
double getOutput(
    double error,
    double minOutput = -infinity(),
    double maxOutput = infinity());
```

Updates the controller state and returns the next output. Each call performs these steps:

1. Add the error to the integral when it is within `integralTolerance`.
2. Optionally reset the integral when the error sign changed.
3. Calculate the derivative from the current and previous errors.
4. Combine the proportional, integral, and derivative terms.
5. Clamp that value between `minOutput` and `maxOutput`.
6. Limit its change from the previous output to `kS`.
7. Update settling time from the current error.

On the first call after construction or `reset`, no previous error exists, so the derivative is equal to the current error.

```cpp
double error = targetHeading - measuredHeading;
double voltage = controller.getOutput(error, -12.0, 12.0);
```

`minOutput` must be less than or equal to `maxOutput`. Because slew limiting occurs after output clamping, the final output can temporarily be outside the requested range when the range does not include the previous output. For example, a positive minimum output may not be reached immediately when ramping up from zero.

### `isSettled`

```cpp
bool isSettled();
```

Returns `true` after the error has remained strictly within `settleTolerance` for at least `settleTime` according to the configured `dtMS` increments.

`isSettled` only reads stored state. Calling it repeatedly does not advance the settling timer; only `getOutput` updates that timer.

### `reset`

```cpp
void reset();
```

Resets the integral, previous error, previous output, and settling timer to zero. It also marks the previous error as unavailable, causing the next derivative term to use the new error directly.

```cpp
controller.reset();
```

## Timing Behavior

This implementation is iteration-based. Its integral and derivative calculations are:

```text
integral += error
derivative = error - previousError
```

The integral is not multiplied by elapsed time, and the derivative is not divided by elapsed time. As a result, changing the update frequency changes controller behavior and usually requires retuning `kI`, `kD`, and `kS`.

Use a consistent loop delay that matches `ExitConditions::dtMS`:

```cpp
constexpr int loopTimeMS = 10;

while (!controller.isSettled()) {
    double error = target - getMeasurement();
    double output = controller.getOutput(error, -12.0, 12.0);

    applyOutput(output);
    vex::task::sleep(loopTimeMS);
}
```

Real robot movements should also have an independent timeout. Settling conditions alone cannot end a loop when a mechanism is blocked, disconnected, or unable to reach its target.

## Complete Motor Example

```cpp
neblib::PID motorController(
    neblib::PID::Gains(0.6, 0.0, 0.15, 0.75),
    neblib::PID::Behaviors(),
    neblib::PID::ExitConditions(1.0, 200, 10));

void moveMotorTo(vex::motor& motor, double targetDegrees)
{
    constexpr int loopTimeMS = 10;
    constexpr int timeoutMS = 2000;
    int elapsedMS = 0;

    motorController.reset();

    while (!motorController.isSettled() && elapsedMS < timeoutMS) {
        double position = motor.position(vex::rotationUnits::deg);
        double error = targetDegrees - position;
        double voltage = motorController.getOutput(error, -12.0, 12.0);

        motor.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
        vex::task::sleep(loopTimeMS);
        elapsedMS += loopTimeMS;
    }

    motor.stop(vex::brakeType::hold);
}
```

The gains in this example are placeholders and must be tuned for the actual mechanism.

## Tuning Guidelines

A basic tuning process is:

1. Set `kI` and `kD` to zero and disable slew limiting.
2. Increase `kP` until the system approaches the target quickly but begins to oscillate.
3. Reduce `kP` until the response is stable.
4. Add `kD` to reduce overshoot and oscillation.
5. Add a small `kI` only if a persistent steady-state error remains.
6. Configure `integralTolerance` to keep the integral from accumulating far from the target.
7. Add `kS` when the output must ramp more gradually.
8. Choose settling conditions that reflect the accuracy and stability the movement actually needs.

Tune using the same units, mechanism load, output limits, and loop interval that will be used on the robot.

## Related Files

- [Utility functions](util.md)
- [Position tracking](position_tracking.md)
- [Documentation conventions](documentation.md)
- [Control-algorithms header](../../include/neblib/control_algorithms.hpp)
- [Control-algorithms implementation](../../src/neblib/control_algorithms.cpp)
