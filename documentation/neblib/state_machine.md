# State Machines and Subsystems

## Overview

The state-machine module provides `neblib::StateMachine<StateEnum>` for organizing mechanism behavior into named states. Each state can have entry, update, and exit callbacks. Only one state is active in each machine.

`neblib::Subsystem<StateEnum>` is an abstract base class that contains a state machine and a clock reference. Concrete subsystems provide their own state registration and periodic behavior.

The caller controls when updates occur. The machine does not create a background task, read the Brain timer, or choose competition-mode behavior automatically.

Lifecycle testing and robot integration are still in progress. This page describes the current implementation; it does not establish runtime or hardware verification.

## Requirements

- Use C++11 and the project's VEX V5 SDK. The current headers include `vex.h`.
- Use an enum type for the states, and register each state before attempting to enter it.
- Supply a persistent `uint32_t` clock variable containing time in milliseconds. That variable must outlive the machine.
- Refresh the clock before calling `periodic()` and before lifecycle operations that need a current entry timestamp.
- Run machine operations and clock updates on one owning thread.
- Keep callbacks nonblocking so other mechanism updates can run promptly.

The public headers are `neblib/state_machine/state_machine.hpp` and `neblib/state_machine/subsystem.hpp`. Template definitions are in the headers so they are available when a machine is instantiated with a particular state enum.

## State and Callback Types

### `StateMachine<StateEnum>`

`StateEnum` identifies the machine's states. Each machine maintains its own registrations, current state, pending transition, and entry timestamp.

There are no built-in states or special enum names. A state named `IDLE`, `NONE`, or `DISABLED` has only the behavior supplied by its callbacks. The caller implements transition conditions, timeouts, and recovery behavior.

### `StateCallbacks`

The nested `StateCallbacks` type contains three public `std::function<void()>` members:

| Member | When it runs |
| --- | --- |
| `entry` | When the machine starts or enters the state through a transition or reset |
| `update` | When `periodic()` updates the active state |
| `exit` | When the machine leaves the state through a transition, reset, or stop |

Its constructors are:

```cpp
StateCallbacks(std::function<void()> en, std::function<void()> up, std::function<void()> ex);
StateCallbacks() = default;
```

The default member values are callbacks that do nothing. Explicitly empty `std::function` values are also supported and are skipped when the corresponding callback would run.

Callbacks take no arguments and return no value. They can access mechanism data through captures. Objects referred to by those captures must remain valid while the callbacks can execute.

### `Registration<State>`

The nested `Registration<State>` template groups two public members: a `State state` identifier and a `StateCallbacks stateCallbacks` value.

```cpp
Registration(State state, StateCallbacks stateCallbacks);
Registration() = default;
```

Use the machine's `StateEnum` as the registration's `State` argument. When default-constructing a registration, explicitly initialize its `state` member before registering it.

## Constructors

### Initial Registration and Clock

```cpp
StateMachine(
    Registration<StateEnum> initialState,
    uint32_t& robotTime);
```

Creates a stopped machine, registers the supplied state and callbacks, and selects that state as current. Construction does not execute callbacks. The entry timestamp initially equals zero.

**Parameters**

- `initialState` - The initial state's identifier and callbacks.
- `robotTime` - A reference to the caller's millisecond clock variable.

Additional states can be registered while the machine remains stopped.

### Registration Map, Initial State, and Clock

```cpp
StateMachine(
    std::map<StateEnum, Registration<StateEnum>> registrations,
    StateEnum initialState,
    uint32_t& robotTime);
```

Creates a stopped machine from a map of registrations and selects `initialState` as current. Construction does not execute callbacks, and the entry timestamp initially equals zero.

**Parameters**

- `registrations` - State registrations to copy into the machine. An empty map is allowed.
- `initialState` - The enum value selected as current, whether or not it has been registered yet.
- `robotTime` - A reference to the caller's millisecond clock variable.

The constructor registers each map value using that registration's `state` member. Use matching map keys and registration identifiers. Duplicate registration identifiers are rejected rather than replacing earlier callbacks.

The empty-map form supports registering states after the machine member has been constructed. Register the selected state before calling `start()` or no-argument `reset()`. Alternatively, `reset(state)` can start the machine in a different registered state.

`StateMachine` has no default constructor.

## StateMachine Methods

### `registerState`

```cpp
bool registerState(StateEnum state,
    std::function<void()> entry,
    std::function<void()> update,
    std::function<void()> exit);

bool registerState(Registration<StateEnum> registration);
```

Registers a state and its callbacks. The first overload accepts them separately; the second accepts a registration object. Neither overload executes callbacks or changes the selected state.

**Returns:** `true` when registration succeeds. Returns `false` without changing registrations if the identifier is already registered or the machine is running.

Duplicate registration does not replace existing callbacks. There is no public operation to remove a registration.

### `start`

```cpp
void start();
```

Starts a stopped machine in its current state. It captures the referenced clock value, marks the machine running, and executes the current state's entry callback. Update runs on a later call to `periodic()`.

Does nothing if the machine is already running, the current state is unregistered, or the call occurs inside one of this machine's callbacks. There is no success return value.

Because the machine is running before entry executes, entry can request a future transition.

### `periodic`

```cpp
void periodic();
```

Performs one update cycle. If a transition is pending, the machine performs it before calling the active state's update callback:

1. Clear the old pending-request flag.
2. Execute the old state's exit callback.
3. Select the destination as current.
4. Capture the referenced clock value as the new entry timestamp.
5. Execute the destination's entry callback.
6. Execute the destination's update callback.

At most one transition is processed per call. Accepted transition requests generated by exit, entry, or update callbacks remain pending for a later cycle.

Does nothing when stopped or called recursively from one of this machine's callbacks. All callbacks execute synchronously on the thread calling `periodic()`.

### `requestStateChange`

```cpp
bool requestStateChange(StateEnum state);
```

Requests a transition to a registered state. A successful request does not immediately execute callbacks; `periodic()` processes the request later.

**Returns:**

- `false` if stopped, if the destination is unregistered, or if another request is already pending. These failures do not replace existing pending work.
- `true` if a request for a different state is accepted.
- `true` for the current state when no request is pending. This is a no-op: it does not re-enter the state, refresh the timer, or occupy the pending slot.

The first pending request wins. Even a request for the current state fails while another request occupies the slot. Use `reset()` when same-state re-entry is needed.

This method is allowed inside callbacks. During exit, the old state is still current; during entry and update, the destination is current.

### `stop`

```cpp
void stop();
```

If running, executes the current state's exit callback once. It then marks the machine stopped and discards pending work, including requests produced during that exit callback.

Repeated calls while stopped do not execute exit again. Later `periodic()` calls do nothing until startup or reset. The selected state and its stored entry timestamp are retained.

Does nothing when called inside one of this machine's callbacks. It has no success return value. Hardware shutdown behavior must be implemented in the callbacks or the robot's mode-handling code.

### `reset(StateEnum state)`

```cpp
bool reset(StateEnum state);
```

Immediately enters the supplied registered state, including when the machine was stopped or the destination is already current.

After validation, it discards old pending work, exits the current state if running, selects the destination, marks the machine running, captures the clock, and executes entry. The reset operation itself does not call update. When called directly, it leaves update to the next `periodic()` call. When `periodic()` uses this overload to process a pending transition, that periodic cycle calls the destination's update after reset returns.

Accepted requests produced by exit or entry survive for a later periodic cycle, with the first pending request taking priority.

**Returns:** `true` after resetting. Returns `false` without mutation if the destination is unregistered or the call occurs inside one of this machine's callbacks.

### `reset()`

```cpp
bool reset();
```

Resets to the current state. It discards old pending work before exit, exits if running, captures a fresh timestamp, executes entry, and leaves the machine running. If stopped, entry runs without a preceding exit.

Accepted callback-generated requests remain pending for a later cycle. Reset does not execute update.

**Returns:** `true` after resetting. Returns `false` without mutation if the current state is unregistered or the call occurs inside one of this machine's callbacks.

### Inspection and Timestamp Methods

| Signature | Behavior |
| --- | --- |
| `StateEnum getCurrentState()` | Returns the selected state, including while stopped or before its initial registration. |
| `StateEnum getPendingState()` | Returns the stored pending-state value. It does not report whether a request is pending. |
| `uint32_t getStartTime()` | Returns the stored entry timestamp, initially zero and normally refreshed by start or reset. |
| `void setStartTime(uint32_t time)` | Directly replaces the stored timestamp without executing callbacks or changing the state. |

`getPendingState()` can return an old value after a transition, reset, or stop. It cannot establish that a transition will occur. The current API has no public query for the pending-request flag or the running/stopped flag.

The inspection methods are not currently declared `const`. `setStartTime()` is public and is not subject to the lifecycle recursion guard; changing it directly changes the basis for elapsed-time calculations.

## Subsystem Base

`neblib::Subsystem<StateEnum>` groups a machine with the common interface for a mechanism:

```cpp
Subsystem(uint32_t& robotTime, StateEnum initialState);
virtual void periodic() = 0;
virtual void registerStates() = 0;
```

`periodic()` and the constructor are public. `registerStates()` is protected. The base also exposes a public `StateMachine<StateEnum> stateMachine` member and a protected `uint32_t& robotTime` reference.

The base constructor creates an empty, stopped machine with the chosen initial enum. It does not call the virtual registration hook or start the machine.

A concrete subsystem should:

1. Inherit publicly when callers need the base interface.
2. Supply the clock and initial enum to the base constructor.
3. Register its states after the base and its own members have been constructed.
4. Implement `periodic()` and explicitly call `stateMachine.periodic()` as part of that update.
5. Start the machine when the objects used by its callbacks are ready.

The base does not automatically forward periodic updates or enforce hardware behavior. It also does not declare a virtual destructor; do not delete a derived subsystem through a base pointer.

The [template subsystem](../../src/neblib/state_machine/template_subsystem.cpp) shows the current registration and update pattern. Its callbacks are empty, and it starts during construction. When callbacks depend on other robot objects, arrange startup after those objects are ready.

## Worked Lifecycle Example

Consider a stopped machine with states A, B, and C registered and A selected. Each callback records its state and callback name in a log. The following is an expected trace, not a captured test result.

| Action | Expected callback events |
| --- | --- |
| Construct and register | None |
| Call `start()` | A entry |
| Call `start()` again | None |
| Call `periodic()` | A update |
| Request B | None; request returns `true` |
| Request C before updating | None; request returns `false` because B is pending |
| Call `periodic()` | A exit, B entry, B update |
| Call `periodic()` again | B update |
| Call no-argument `reset()` | B exit, B entry |
| Call `stop()` | B exit |
| Call `periodic()` while stopped | None |

To check callback-generated requests, start a separate scenario with A active and B pending. Let A's exit request C. The first periodic cycle must run A exit, B entry, and B update. The next cycle must run B exit, C entry, and C update. This checks that consuming B frees the request slot before exit and that C is deferred.

Repeat this kind of check for requests from entry and update, invalid destinations, same-state requests, reset clearing old work, and stop discarding exit-generated requests. Exact event logs and return values provide stronger evidence than visual observation alone.

## Timing and Update Ownership

The clock reference refers to a variable, not to the timer function used to initialize it. Reading the Brain timer once does not make that variable advance automatically.

For each owning-loop iteration, refresh the millisecond clock first, then update the machines in a consistent order, then yield until the next iteration. A loop interval of approximately 10 ms is the current design target, not a requirement enforced by `StateMachine`.

Entry timestamps reflect the most recent value supplied by the caller. If the clock is stale when start or reset occurs, the captured timestamp is stale too. Exit runs before the new timestamp is captured, so it can still inspect the previous state's entry time.

To calculate elapsed time, subtract the entry timestamp from the current clock using unsigned 32-bit arithmetic. This handles one timer rollover provided the actual measured interval is shorter than the full counter period. Avoid signed timestamp conversions and direct comparisons with an entry-time-plus-duration sum across rollover. There is no built-in elapsed-time getter or automatic timeout transition.

Use a manually advanced clock for lifecycle tests so timing expectations do not depend on sleeps or hardware scheduling. Check timestamp refresh on start and reset, retention across ordinary updates, and elapsed time across rollover.

## Callback and Integration Constraints

- Callbacks run inline. Sleeps, sensor-wait loops, and blocking drivetrain routines delay every later operation on the owning loop.
- The callback guard prevents recursive lifecycle execution on the same machine. It is not a mutex and does not make concurrent access safe. Keep requests, lifecycle calls, inspection, and clock writes on the owner thread; use a synchronized handoff for commands from other tasks.
- Callbacks may inspect the machine and request transitions. Calls to `periodic()`, `start()`, `stop()`, and either reset overload are rejected while one of that machine's callbacks executes.
- Callback storage copies function objects. Referenced objects and captured pointers must remain valid. Copying or moving a machine or subsystem does not retarget callbacks that capture an object's address; the current classes do not prohibit these operations.
- Callbacks should not throw. The callback guard is not restored if a callback exits through an exception, and the project's VEX build disables exceptions.
- Competition-mode changes need explicit handling. The machine does not automatically stop when disabled, discard commands at mode changes, or select hardware outputs. Destruction also does not call `stop()` or execute exit.

## Related Files

- [State-machine header](../../include/neblib/state_machine/state_machine.hpp)
- [Subsystem base header](../../include/neblib/state_machine/subsystem.hpp)
- [Template subsystem](../../src/neblib/state_machine/template_subsystem.cpp)
- [Control algorithms](control_algorithms.md)
- [Documentation conventions](documentation.md)
- [Documentation topics](../README.md)
