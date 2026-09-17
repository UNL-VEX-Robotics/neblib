# Autonomous Selector

## Overview

`neblib::AutonomousSelector` lets a driver select an autonomous route and alliance color on the VEX V5 Brain Screen before a match. Supply a list of route names, call `run()`, and read the selection with `getRoute()` and `getColor()`.

The selector stores names and selection state. Your competition code is responsible for running the selected routine and performing any sensor calibration.

## Requirements

- Include `neblib/auton_selector.hpp` in your program.
- Provide the global VEX `Brain` object declared by the project's `vex.h`.
- Supply at least one route name before calling `run()`.
- Reserve the Brain Screen for the selector while it runs; other tasks drawing to the same screen can overwrite it.

## API Reference

### `AutonomousSelector`

```cpp
AutonomousSelector(std::vector<std::string> routes);
```

Constructs a selector with the first route (index `0`) and the red alliance selected. Construction does not draw anything or start a task.

**Parameters**

- `routes` — Route names in selection order. The selector owns the names; passing an existing vector copies it, so later edits to that vector do not update the selector.

Use short, distinct names. Displayed names longer than 20 string bytes are shortened to the first 17 bytes followed by `...`. Stored names and route indices are unchanged. This is a byte limit, not a guarantee that every name fits between the arrows; multibyte text can be cut in the middle of a character.

### `run`

```cpp
void run();
```

Displays the selector and blocks the calling task until the user presses **Calibrate**.

| Control | Action |
| --- | --- |
| Left arrow | Selects the previous route; wraps from the first to the last. |
| Right arrow | Selects the next route; wraps from the last to the first. |
| Color | Toggles between red and blue and updates the button's fill color. |
| Calibrate | Confirms the current selection, fills the screen with the alliance color, prints `Route: ` and the selected name, and returns. |

A held touch can repeat an action after 500 milliseconds. Releasing the screen allows the next touch to be handled sooner. Pressing outside the buttons does not change the selection.

**Notes:** Despite its label, **Calibrate** only confirms the selection and updates the screen. It does not calibrate a sensor or execute a route. Calling `run()` again opens the selector with the previous route and alliance still selected.

### `getColor`

```cpp
vex::color getColor();
```

**Returns:** The current alliance, either `vex::color::red` or `vex::color::blue`. The initial value is red.

### `getRoute`

```cpp
int getRoute();
```

**Returns:** The zero-based index of the current route in the original list. The initial value is `0`. For a nonempty list, the index remains between `0` and the number of routes minus one.

The result is an index, not a route name or a callable routine. Keep the mapping between indices and robot routines consistent with the order supplied to the constructor.

## Example

This minimal competition example offers `Left` and `Right` routes. Each route function prints its name and alliance so you can verify selection without moving the robot. Replace those function bodies with your team's autonomous routines.

```cpp
#include "vex.h"
#include "neblib/auton_selector.hpp"

vex::brain Brain;
vex::competition Competition;

neblib::AutonomousSelector autoSelector({"Left", "Right"});

void runLeft(vex::color alliance)
{
    Brain.Screen.print("Left route: %s",
                       alliance == vex::color::red ? "red" : "blue");
}

void runRight(vex::color alliance)
{
    Brain.Screen.print("Right route: %s",
                       alliance == vex::color::red ? "red" : "blue");
}

void pre_auton()
{
    autoSelector.run(); // Blocks until Calibrate is pressed.
    // Perform any required sensor calibration here before returning.
}

void autonomous()
{
    vex::color alliance = autoSelector.getColor();
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);

    switch (autoSelector.getRoute())
    {
    case 0: // Left: first name in the constructor's list.
        runLeft(alliance);
        break;
    case 1: // Right: second name in the constructor's list.
        runRight(alliance);
        break;
    }
}

void usercontrol()
{
    while (true)
    {
        // Add your team's driver controls here.
        vex::task::sleep(20);
    }
}

int main()
{
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);
    pre_auton();

    while (true)
    {
        vex::task::sleep(100);
    }
}
```

When adapting this example to an existing competition project, reuse its `Brain` and `Competition` objects and combine the callback bodies with your existing setup; define each object only once. Keep the selector alive for both `pre_auton()` and `autonomous()`, as the global `autoSelector` is here.

Before the match, use the arrows to choose a route, press **Color** if needed, and press **Calibrate** to finish selection. When the competition system starts autonomous mode, the callback reads the saved route index and passes the selected alliance to that route's function. Changing the order of the names requires updating the matching `case` values.

## Behavior and Warnings

- **Finish selection before the match.** When called from `pre_auton()`, `run()` must return before that setup function can finish. There is no timeout or automatic exit when competition mode changes.
- **Always provide a nonempty list.** The current implementation displays `No Routes` for an empty list but then continues into the selection loop and accesses index `0` with `routes.at()`. That access is out of range; the message is not a safe empty-list fallback. `getRoute()` also returns `0` for an empty list, which is not a valid selection.
- **Route and alliance are independent.** Changing the alliance does not filter or reorder routes. Both arrows leave the selection at index `0` when there is only one route.
- **Confirmation leaves a summary on screen.** The screen background shows the selected alliance and the route uses the same shortening rule as the selection screen.

## Related Pages

- [Documentation welcome page](../README.md)
- [Documentation conventions](documentation.md)
- [Autonomous selector header](../../include/neblib/auton_selector.hpp)
- [Autonomous selector implementation](../../src/neblib/auton_selector.cpp)
- [Drawable shapes header](../../include/neblib/shapes.hpp)
