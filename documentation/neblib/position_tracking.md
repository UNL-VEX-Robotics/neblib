# Position Tracking

The position-tracking module estimates a robot's location and orientation as it moves. It provides a common `PositionTracking` interface and an `Odometry` implementation that combines two tracking wheels with a VEX V5 Inertial Sensor.

Include the header before using the module:

```cpp
#include "neblib/position_tracking.hpp"
```

All types on this page are in the `neblib` namespace.

## Coordinate System and Units

neblib represents the robot's position with an `x` coordinate, a `y` coordinate, and a heading:

- At a heading of `0` degrees, positive `y` is forward.
- At a heading of `0` degrees, positive `x` is the direction measured as positive by the perpendicular tracking wheel.
- Heading is measured in degrees using the Inertial Sensor.
- `x`, `y`, tracker-wheel diameter, and tracker-wheel offsets must all use the same distance unit.

For example, if wheel diameter and offsets are measured in inches, the resulting `x` and `y` coordinates are also in inches. The coordinate origin is arbitrary and can be changed with `setPose`.

The direction reported by each tracker depends on the orientation and reversal setting of its Rotation Sensor. Verify that each tracker reports the expected sign before relying on the calculated pose.

## `Pose`

```cpp
struct Pose
{
    double x;
    double y;
    double heading;

    Pose(double x, double y, double heading);
    Pose();
};
```

`Pose` stores a complete position estimate.

### Members

- `x` - The robot's position on the x-axis.
- `y` - The robot's position on the y-axis.
- `heading` - The robot's orientation in degrees.

The three-argument constructor initializes all members from the supplied values. The default constructor initializes all three members to `0.0`.

```cpp
neblib::Pose startingPose(24.0, 12.0, 90.0);
neblib::Pose origin; // x = 0.0, y = 0.0, heading = 0.0
```

## Tracking-Wheel Setup

`Odometry` requires two objects implementing the [`TrackerWheel`](../../include/neblib/devices/tracker_wheel.hpp) interface:

- The **parallel tracker** rolls when the robot moves forward or backward.
- The **perpendicular tracker** rolls when the robot moves sideways.

neblib includes `RotationTrackerWheel`, which converts the revolutions reported by a VEX Rotation Sensor into linear distance:

```cpp
vex::rotation parallelSensor(vex::PORT1);
vex::rotation perpendicularSensor(vex::PORT2);

neblib::RotationTrackerWheel parallelTracker(parallelSensor, 2.75);
neblib::RotationTrackerWheel perpendicularTracker(perpendicularSensor, 2.75);
```

Here, `2.75` is the wheel diameter. If that value is in inches, both tracker positions and the final pose use inches.

Tracker offsets are measured from the robot's center of rotation to the center of each tracking wheel along the respective axis:

- `parallelDistance` is positive when the parallel tracker is to the right of the center of rotation and negative when it is to the left.
- `perpendicularDistance` is positive when the perpendicular tracker is behind the center of rotation and negative when it is in front.

Measure these offsets carefully. Incorrect diameters or offsets cause systematic position error, especially while the robot turns.

## `PositionTracking`

```cpp
class PositionTracking
{
public:
    virtual ~PositionTracking() = default;
    virtual int begin() = 0;
    virtual void stop() = 0;
    virtual void calibrate() = 0;
    virtual void setPose(Pose newPose) = 0;
    virtual void setPose(double x, double y, double heading);
    virtual Pose getPose() = 0;
};
```

`PositionTracking` is the base interface for position-tracking algorithms. Code that only needs a pose can accept a pointer or reference to this type without depending specifically on `Odometry`.

The class is abstract and cannot be constructed directly.

## `Odometry`

`Odometry` estimates movement using arcs. During each update, it reads both tracking wheels and the Inertial Sensor, accounts for tracker movement caused by rotation, and converts the robot-relative change into field coordinates.

The algorithm is based on the [5225 E-Pilons Introduction to Position Tracking document](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf).

### Constructor

```cpp
Odometry(
    TrackerWheel& parallelTrackerWheel,
    double parallelDistance,
    TrackerWheel& perpendicularTrackerWheel,
    double perpendicularDistance,
    vex::inertial& imu);
```

**Parameters**

- `parallelTrackerWheel` - The tracking wheel aligned with forward robot movement.
- `parallelDistance` - Its signed sideways offset from the center of rotation.
- `perpendicularTrackerWheel` - The tracking wheel aligned with sideways robot movement.
- `perpendicularDistance` - Its signed forward or backward offset from the center of rotation.
- `imu` - The VEX V5 Inertial Sensor used to measure the robot's orientation.

The trackers and Inertial Sensor are stored by reference. They must remain alive for the entire lifetime of the `Odometry` object.

```cpp
vex::inertial imu(vex::PORT3);

neblib::Odometry odometry(
    parallelTracker,
    1.5,
    perpendicularTracker,
    2.0,
    imu);
```

In this example, the parallel tracker is `1.5` units to the right of the center and the perpendicular tracker is `2.0` units behind it.

### `calibrate`

```cpp
void calibrate();
```

Resets both tracking-wheel positions and calibrates the Inertial Sensor. The function blocks until Inertial Sensor calibration finishes.

```cpp
odometry.calibrate();
```

Keep the robot stationary on a level surface during calibration. Calibration should normally occur before starting the update loop. Calling it after tracking has begun can make the newly reset sensor readings inconsistent with the previous readings stored by the algorithm.

### `begin`

```cpp
int begin();
```

Starts the position-update loop. The method reads the sensors and updates the pose every `10` milliseconds until `stop` is called.

**Returns:** `0` after the loop stops.

`begin` blocks the calling thread, so it should normally run in its own VEX task:

```cpp
neblib::launchTask(std::bind(&neblib::Odometry::begin, &myOdometryObject))
```


Do not start more than one update loop for the same `Odometry` object.

### `stop`

```cpp
void stop();
```

Requests that the update loop exit. The loop checks this state once per update and `begin` returns afterward.

```cpp
odometry.stop();
```

Stopping the loop does not reset the current pose or sensor positions.

### `setPose`

```cpp
void setPose(Pose newPose);
void setPose(double x, double y, double heading);
```

Replaces the current position estimate. Both overloads also set the Inertial Sensor's heading and rotation to the supplied heading.

**Parameters**

- `newPose` - The complete replacement pose.
- `x` - The new x-coordinate.
- `y` - The new y-coordinate.
- `heading` - The new orientation in degrees.

```cpp
odometry.setPose(0.0, 0.0, 0.0);

// Equivalent:
odometry.setPose(neblib::Pose(0.0, 0.0, 0.0));
```

Call `setPose` after calibration and before `begin` when the robot's starting field position is known. Although pose access is protected by a mutex, `setPose` also changes sensor and internal rotation state; stop the update loop before resetting the pose later in the program. The robot should be stationary when changing the Inertial Sensor state.

### `getPose`

```cpp
Pose getPose();
```

Returns a copy of the latest position estimate. Access is protected by a mutex so the update task cannot modify the pose while it is being copied.

```cpp
neblib::Pose currentPose = odometry.getPose();

Brain.Screen.printAt(
    10,
    40,
    "x: %.2f, y: %.2f, heading: %.2f",
    currentPose.x,
    currentPose.y,
    currentPose.heading);
```

## Complete Setup Example

The devices and odometry object should normally have a lifetime that covers the entire program:

```cpp
#include "vex.h"
#include "neblib/position_tracking.hpp"

vex::rotation parallelSensor(vex::PORT1);
vex::rotation perpendicularSensor(vex::PORT2);
vex::inertial imu(vex::PORT3);

neblib::RotationTrackerWheel parallelTracker(parallelSensor, 2.75);
neblib::RotationTrackerWheel perpendicularTracker(perpendicularSensor, 2.75);

neblib::Odometry odometry(
    parallelTracker,
    1.5,
    perpendicularTracker,
    2.0,
    imu);

int main()
{
    odometry.calibrate();
    odometry.setPose(0.0, 0.0, 0.0);

    vex::task odometryTask = neblib::launchTask([]() {
        odometry.begin();
    });

    while (true) {
        vex::wait(100, vex::msec);
    }
}
```

The example offsets are placeholders. Replace the wheel diameters, sensor ports, reversal settings, and offsets with measurements from the actual robot.

## Accuracy and Troubleshooting

Position tracking accumulates sensor and measurement error over time. For the best results:

- Measure the effective tracking-wheel diameter through testing rather than relying only on its nominal size.
- Measure offsets from the actual center of rotation.
- Prevent tracking wheels from slipping or losing contact with the field.
- Make sure each Rotation Sensor reports the correct direction.
- Keep the robot still while the Inertial Sensor calibrates.
- Use one consistent distance unit throughout the setup.
- Route and mount the perpendicular tracker so forward motion does not cause unwanted rotation.

If forward movement changes `x` instead of `y`, the trackers may be passed to the constructor in the wrong order. If a coordinate moves in the opposite direction from what is expected, reverse the corresponding Rotation Sensor or correct its mounting. If errors are greatest during turns, check the signed tracker offsets.

## Related Files

- [Utility functions](util.md)
- [Documentation conventions](documentation.md)
- [Position-tracking header](../../include/neblib/position_tracking.hpp)
- [Position-tracking implementation](../../src/neblib/position_tracking.cpp)
- [Tracker-wheel header](../../include/neblib/devices/tracker_wheel.hpp)
