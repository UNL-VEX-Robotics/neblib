# Utility Functions

The utilities in [`util.hpp`](../../include/neblib/util.hpp) provide small, reusable operations used throughout neblib. They include task launching, numeric helpers, angle conversion, random-number generation, and case-insensitive string searching.

Include the header before using these functions:

```cpp
#include "neblib/util.hpp"
```

All utilities are in the `neblib` namespace.

## Task Utility

### `launchTask`

```cpp
template <class F>
vex::task launchTask(F&& function);
```

Creates and starts a VEX task that invokes `function`. This helper makes it possible to launch a capturing lambda or another callable object as a `vex::task`. This allows users to launch a task to an object-owned function.

**Parameters**

- `function` — A callable that takes no arguments and can be stored as a `std::function<void()>`.

**Returns:** The newly created `vex::task`.

```cpp
int count = 5;

vex::task worker = neblib::launchTask([count]() {
    for (int i = 0; i < count; ++i) {
        // Perform background work.
        vex::wait(20, vex::msec);
    }
});
```

The task begins running when it is created. Values needed by the task should usually be captured by value. If a lambda captures local variables by reference, those variables must remain alive for as long as the task uses them. Keep or otherwise manage the returned task when later task control is required.

## Numeric Utilities

### `sign`

```cpp
template <typename T>
int sign(T num);
```

Determines whether a number is negative, positive, or zero.

**Parameters**

- `num` — A value that can be compared with zero.

**Returns:** `-1` when `num` is negative, `1` when it is positive, and `0` otherwise.

```cpp
int direction = neblib::sign(-4.5); // -1
```

### `clamp`

```cpp
double clamp(double num, double min, double max);
```

Restricts a number to the inclusive range from `min` to `max`.

**Parameters**

- `num` — The value to restrict.
- `min` — The lowest allowed result.
- `max` — The highest allowed result.

**Returns:** `min` when `num` is below the range, `max` when it is above the range, or `num` when it is already within the range.

```cpp
double motorVoltage = neblib::clamp(requestedVoltage, -12.0, 12.0);
```

`min` must be less than or equal to `max`.

### `wrap`

```cpp
double wrap(double num, double min, double max);
```

Moves a number into the range from `min` to `max` by repeatedly adding or subtracting the range width. This is useful for cyclic values such as angles.

**Parameters**

- `num` — The value to wrap.
- `min` — The lower boundary.
- `max` — The upper boundary.

**Returns:** The equivalent value within the range, including either boundary.

```cpp
double error = neblib::wrap(targetHeading - currentHeading, -180.0, 180.0);
```

Unlike a half-open wrapping interval, this implementation leaves a value equal to `max` unchanged. For example, `wrap(180.0, -180.0, 180.0)` returns `180.0`. `max` must be greater than `min`; an empty or reversed range can prevent the function from terminating.

## Angle Conversion

### `toRad`

```cpp
double toRad(double degrees);
```

Converts an angle from degrees to radians.

**Parameters**

- `degrees` — The angle in degrees.

**Returns:** The equivalent angle in radians.

```cpp
double radians = neblib::toRad(90.0); // Approximately 1.5708
```

### `toDeg`

```cpp
double toDeg(double radians);
```

Converts an angle from radians to degrees.

**Parameters**

- `radians` — The angle in radians.

**Returns:** The equivalent angle in degrees.

```cpp
double degrees = neblib::toDeg(M_PI); // Approximately 180.0
```

These functions convert units only; they do not restrict or wrap the resulting angle.

## Random-Number Utilities

The random-number functions share a generator that is initialized using `std::random_device`. Results therefore vary between runs when the platform provides nondeterministic seed data. neblib does not currently expose a way to set a repeatable seed.

### `gaussRandom`

```cpp
double gaussRandom(double mean, double stddev);
```

Generates a value from a Gaussian, also called normal, distribution.

**Parameters**

- `mean` — The center or expected value of the distribution.
- `stddev` — The standard deviation. This value must be positive.

**Returns:** A randomly generated `double` from the requested distribution.

```cpp
double measurementNoise = neblib::gaussRandom(0.0, 0.25);
```

The result is not limited to a fixed minimum or maximum. Values farther from the mean become less likely, but remain possible.

### `uniformRandom`

```cpp
double uniformRandom(double min, double max);
```

Generates a floating-point value using a uniform distribution, giving values across the requested range equal probability density.

**Parameters**

- `min` — The lower boundary of the distribution.
- `max` — The upper boundary of the distribution.

**Returns:** A randomly generated `double` in the requested range.

```cpp
double startingAngle = neblib::uniformRandom(0.0, 360.0);
```

`min` must be less than or equal to `max`.

## String Utility

### `contains`

```cpp
bool contains(const char* str, const char* substr);
```

Performs a case-insensitive search for a substring within a null-terminated C string.

**Parameters**

- `str` — The string to search.
- `substr` — The substring to find.

**Returns:** `true` when `substr` occurs within `str`; otherwise, `false`.

```cpp
bool isBlueAuton = neblib::contains("Blue Left", "blue"); // true
```

The comparison ignores letter case, so `"BLUE"`, `"Blue"`, and `"blue"` match one another. An empty substring matches every non-null string. The function returns `false` if either pointer is `nullptr`.

Both arguments must point to valid null-terminated strings for the duration of the call.

## Related Files

- [Documentation conventions](documentation.md)
- [Utility header](../../include/neblib/util.hpp)
- [Utility implementation](../../src/neblib/util.cpp)
