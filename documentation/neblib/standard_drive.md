# Standard Drive

`StandardDrive` controls a conventional differential-drive robot with left and right motor groups. It supports tank and arcade driver control as well as PID-controlled driving, point turns, and swing turns.

Include the header before using the class:

```cpp
#include "neblib/standard_drive.hpp"
```

`StandardDrive` is in the `neblib` namespace.

## Requirements and Conventions

The class requires:

- a left `vex::motor_group`;
- a right `vex::motor_group`;
- a `PositionTracking` pointer;
- a parallel `TrackerWheel` for measuring forward distance; and
- a VEX V5 Inertial Sensor for measuring turns and heading.

The current implementation stores the `PositionTracking` pointer but does not use it in any drive method. The pointer may therefore be `nullptr` for the methods documented on this page, although a valid object may be required by future versions.

The following conventions apply:

- Relative and absolute turn measurements use degrees.
- Drive distance uses the same unit returned by `parallelTrackerWheel.getPosition()`.
- Autonomous motor outputs use volts.
- Autonomous methods update every `10` milliseconds.
- Timeout arguments and return values use integer milliseconds.
- Autonomous methods block until their main PID settles or the timeout expires.
- Autonomous methods stop both motor groups using `vex::brakeType::hold` before returning.

Motor reversal should be configured when constructing the individual VEX motors. Positive power must make both sides drive the robot forward for the control calculations to behave as intended.

## Constructor

```cpp
StandardDrive(
    vex::motor_group&& leftMotors,
    vex::motor_group&& rightMotors,
    PositionTracking* positionTracking,
    TrackerWheel& parallelTrackerWheel,
    vex::inertial& imu);
```

Creates a standard-drive controller.

**Parameters**

- `leftMotors` - Motor group controlling the left side of the drivetrain.
- `rightMotors` - Motor group controlling the right side of the drivetrain.
- `positionTracking` - Pointer to a position-tracking implementation. It is stored but currently unused.
- `parallelTrackerWheel` - Tracker wheel aligned with forward robot movement.
- `imu` - Inertial Sensor used for relative rotation and absolute heading.

The constructor accepts the motor groups as rvalue references, so named motor groups must be passed with `std::move`. The tracker wheel and Inertial Sensor are stored by reference and must remain alive for the lifetime of the `StandardDrive` object.

```cpp
vex::motor leftFront(vex::PORT1, true);
vex::motor leftBack(vex::PORT2, true);
vex::motor rightFront(vex::PORT3, false);
vex::motor rightBack(vex::PORT4, false);

vex::motor_group leftMotors(leftFront, leftBack);
vex::motor_group rightMotors(rightFront, rightBack);

vex::rotation parallelRotation(vex::PORT5);
vex::inertial imu(vex::PORT6);
neblib::RotationTrackerWheel parallelTracker(parallelRotation, 2.75);

neblib::StandardDrive drive(
    std::move(leftMotors),
    std::move(rightMotors),
    nullptr,
    parallelTracker,
    imu);
```

Ports, motor reversal, and wheel diameter in this example are placeholders and must match the actual robot.

## Configuring Controllers

`StandardDrive` does not create PID controllers automatically. The controller pointers initially equal `nullptr` and must be assigned before calling the corresponding autonomous methods.

```cpp
void setTurnPID(PID* turnPID);
void setLinearPID(PID* linearPID);
void setAngularPID(PID* angularPID);
void setSwingPID(PID* swingPID);
```

- `setTurnPID` configures `turnFor` and `turnTo`.
- `setLinearPID` configures the distance portion of `driveFor`.
- `setAngularPID` configures heading correction during `driveFor`.
- `setSwingPID` configures `swingFor` and `swingTo`.

Passing `nullptr`, or calling an autonomous method before assigning its required controller, causes that method to dereference a null pointer. Controller objects must also remain alive while the `StandardDrive` object uses them.

Each controller should use an `ExitConditions::dtMS` value of `10` to match the drive loops:

```cpp
neblib::PID turnPID(
    neblib::PID::Gains(0.08, 0.0, 0.2, 0.5),
    neblib::PID::Behaviors(),
    neblib::PID::ExitConditions(1.0, 200, 10));

neblib::PID linearPID(
    neblib::PID::Gains(0.7, 0.0, 0.1, 0.5),
    neblib::PID::Behaviors(),
    neblib::PID::ExitConditions(0.5, 200, 10));

neblib::PID angularPID(
    neblib::PID::Gains(0.08, 0.0, 0.15),
    neblib::PID::Behaviors(),
    neblib::PID::ExitConditions(1.0, 200, 10));

neblib::PID swingPID(
    neblib::PID::Gains(0.08, 0.0, 0.2, 0.5),
    neblib::PID::Behaviors(),
    neblib::PID::ExitConditions(1.0, 200, 10));

drive.setTurnPID(&turnPID);
drive.setLinearPID(&linearPID);
drive.setAngularPID(&angularPID);
drive.setSwingPID(&swingPID);
```

These gains are examples only. Tune each controller for the actual drivetrain, load, units, and battery behavior.

## Driver-Control Methods

### `tankDrive`

```cpp
void tankDrive(
    double leftInput,
    double rightInput,
    vex::velocityUnits unit = vex::velocityUnits::pct);

void tankDrive(
    double leftInput,
    double rightInput,
    vex::voltageUnits unit = vex::voltageUnits::volt);
```

Sends independent inputs to the left and right motor groups.

```cpp
drive.tankDrive(
    Controller1.Axis3.position(),
    Controller1.Axis2.position(),
    vex::velocityUnits::pct);
```

The overload is selected by the unit argument. When no unit is provided, inputs are interpreted as velocity percentages.

### `arcadeDrive`

```cpp
void arcadeDrive(
    double linearInput,
    double angularInput,
    vex::velocityUnits unit = vex::velocityUnits::pct);

void arcadeDrive(
    double linearInput,
    double angularInput,
    vex::voltageUnits unit = vex::voltageUnits::volt);
```

Combines forward and turning inputs using:

```text
left output  = linearInput + angularInput
right output = linearInput - angularInput
```

```cpp
drive.arcadeDrive(
    Controller1.Axis3.position(),
    Controller1.Axis1.position(),
    vex::velocityUnits::pct);
```

The combined values are passed directly to the VEX motor groups. `StandardDrive` does not normalize or clamp the sum, so combined inputs may exceed the nominal range.

### `stop`

```cpp
void stop(vex::brakeType stopType = vex::brakeType::hold);
```

Stops both motor groups using the requested braking mode.

```cpp
drive.stop();                       // Hold by default
drive.stop(vex::brakeType::coast); // Coast instead
```

## Point Turns

A point turn drives the left and right sides in opposite directions so the robot rotates near its center.

### `turnFor`

```cpp
int turnFor(
    double degrees,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int turnFor(
    double degrees,
    int timeout = std::numeric_limits<int>::max());
```

Turns by a relative number of degrees using `imu.rotation()` and the configured turn PID.

**Parameters**

- `degrees` - Relative change in orientation, in degrees.
- `minOutput` - Minimum turn-controller output in volts.
- `maxOutput` - Maximum turn-controller output in volts.
- `timeout` - Maximum duration in milliseconds.

**Returns:** Approximate elapsed time in milliseconds, counted in `10`-millisecond loop increments.

```cpp
drive.turnFor(90.0, -8.0, 8.0, 2000);
```

This method uses the Inertial Sensor's continuous rotation value, allowing a relative target beyond a single `0`-to-`360`-degree range.

### `turnTo`

```cpp
int turnTo(
    double heading,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int turnTo(
    double heading,
    int timeout = std::numeric_limits<int>::max());
```

Turns to an absolute Inertial Sensor heading. The heading error is wrapped between `-180` and `180` degrees, so the robot normally takes the shortest turn.

**Parameters**

- `heading` - Target absolute heading in degrees.
- `minOutput` - Minimum turn-controller output in volts.
- `maxOutput` - Maximum turn-controller output in volts.
- `timeout` - Maximum duration in milliseconds.

**Returns:** Approximate elapsed time in milliseconds.

```cpp
drive.turnTo(270.0, -8.0, 8.0, 2000);
```

Both point-turn methods reset the turn PID before moving and require `setTurnPID` to have been called.

## Driving a Distance

### `driveFor`

```cpp
int driveFor(
    double distance,
    double heading,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int driveFor(
    double distance,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int driveFor(
    double distance,
    double heading,
    int timeout = std::numeric_limits<int>::max());

int driveFor(
    double distance,
    int timeout = std::numeric_limits<int>::max());
```

Drives a relative distance measured by the parallel tracker while using the angular PID to maintain a heading.

**Parameters**

- `distance` - Relative travel distance in the tracker wheel's configured distance unit. Positive and negative values travel in opposite directions.
- `heading` - Absolute heading to maintain, in degrees.
- `minOutput` - Minimum linear-controller output in volts.
- `maxOutput` - Maximum linear-controller output in volts.
- `timeout` - Maximum duration in milliseconds.

**Returns:** Approximate elapsed time in milliseconds.

When `heading` is omitted, the robot captures and holds its current heading at the start of the call. When output limits are omitted, the controller receives infinite limits.

```cpp
drive.driveFor(24.0, 0.0, -10.0, 10.0, 3000);

// The one-argument form holds the current heading with no finite timeout.
drive.driveFor(12.0);
```

Timeouts are `int` values while headings and output limits are `double`. Use an integer literal such as `2000` for a timeout; this allows the compiler to select the intended overload.

`driveFor` resets both the linear and angular PIDs, but only the linear PID determines when the movement is settled. It requires both `setLinearPID` and `setAngularPID` to have been called.

The angular correction is limited to `-12` through `12` volts before being added to or subtracted from the linear output. The final left and right sums are not clamped by `StandardDrive`, so they can exceed the nominal `-12`-to-`12`-volt range before being passed to the VEX API.

## Swing Turns

A swing turn holds one side of the drivetrain and drives the other side. All swing methods require a controller assigned with `setSwingPID`.

For `vex::turnType::right`, the right motor group is held and the left group moves. For any other `turnType`, the left group is held and the right group moves.

### `swingFor`

```cpp
int swingFor(
    vex::turnType direction,
    double degrees,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int swingFor(
    vex::turnType direction,
    double degrees,
    int timeout = std::numeric_limits<int>::max());
```

Performs a relative swing turn using the Inertial Sensor's continuous rotation value.

**Parameters**

- `direction` - Swing-turn direction and the side of the drivetrain that remains held.
- `degrees` - Relative magnitude of the turn in degrees.
- `minOutput` - Minimum swing-controller output in volts.
- `maxOutput` - Maximum swing-controller output in volts.
- `timeout` - Maximum duration in milliseconds.

**Returns:** Approximate elapsed time in milliseconds.

```cpp
drive.swingFor(vex::turnType::right, 45.0, -8.0, 8.0, 2000);
```

### `swingTo` with a Forced Rotation Direction

```cpp
int swingTo(
    vex::turnType turnDirection,
    vex::directionType direction,
    double heading,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int swingTo(
    vex::turnType turnDirection,
    vex::directionType direction,
    double heading,
    int timeout = std::numeric_limits<int>::max());
```

Swings to an absolute heading while forcing the wrapped error into a particular direction:

- `vex::directionType::fwd` wraps the error from `0` through `360` degrees.
- `vex::directionType::rev` wraps the error from `-360` through `0` degrees.

`turnDirection` chooses which side remains held. `direction` chooses the allowed sign and path of the heading error.

```cpp
drive.swingTo(
    vex::turnType::right,
    vex::directionType::fwd,
    180.0,
    -8.0,
    8.0,
    3000);
```

### `swingTo` Using the Shortest Rotation

```cpp
int swingTo(
    vex::turnType turnDirection,
    double heading,
    double minOutput,
    double maxOutput,
    int timeout = std::numeric_limits<int>::max());

int swingTo(
    vex::turnType turnDirection,
    double heading,
    int timeout = std::numeric_limits<int>::max());
```

Swings to an absolute heading using an error wrapped from `-180` through `180` degrees. This normally selects the shortest rotation.

```cpp
drive.swingTo(vex::turnType::left, 90.0, -8.0, 8.0, 2000);
```

## Timeouts and Completion

Each autonomous method checks two exit conditions:

1. The primary PID reports that it is settled.
2. The elapsed time reaches `timeout`.

The methods do not report which condition ended the movement. They return only their approximate elapsed time. Compare the return value with the requested timeout if the caller needs to infer that a timeout may have occurred.

Always provide a finite timeout for robot movements. The default is `std::numeric_limits<int>::max()`, which effectively disables the timeout for a normal robot run. A blocked drivetrain, failed sensor, incorrectly tuned PID, or unreachable target can therefore keep a method running for a very long time when the timeout is omitted.

```cpp
int elapsed = drive.turnTo(90.0, -8.0, 8.0, 2000);

if (elapsed >= 2000) {
    // The movement may have timed out.
}
```

## Setup Checklist

Before running an autonomous movement:

1. Configure motor reversal so positive commands drive both sides forward.
2. Calibrate the Inertial Sensor while the robot is stationary.
3. Reset or verify the parallel tracker position.
4. Assign every PID required by the intended method.
5. Tune PID exit conditions for a `10`-millisecond update interval.
6. Use output limits appropriate for VEX voltage control.
7. Supply a finite timeout.

## Related Files

- [Control algorithms](control_algorithms.md)
- [Position tracking](position_tracking.md)
- [Utility functions](util.md)
- [Standard-drive header](../../include/neblib/standard_drive.hpp)
- [Standard-drive implementation](../../src/neblib/standard_drive.cpp)
