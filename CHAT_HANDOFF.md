# neblib state-machine project: conversation handoff

Prepared October 21, 2026 (America/Chicago). This is a curated continuation document, not a verbatim chat transcript. It preserves the user's goals, decisions, learning preferences, and current evidence. Read the current repository before treating any finding as still applicable.

## Instructions for the next assistant

The user is learning C++ and explicitly wants to implement the code personally. Help through explanations, design discussions, reviews, and compiler diagnostics. Do not edit or generate replacement source implementations unless the user changes that instruction. Small syntax illustrations have been useful when requested; explain their meaning rather than handing over completed classes.

The repository README explicitly prohibits AI-generated code in neblib. Treat that as a repository requirement. Its statements about competition policies were not independently verified during this work; do not present them as newly verified competition rules.

When the user says “check again” or “help debug,” read the latest files, inspect the actual implementation, and explain concrete errors. Distinguish compiler evidence from source-level reasoning and from runtime/hardware tests. Never say the system is robust just because the template compiles. Preserve the user's work. Do not repeat outdated issues that have already been fixed.

The immediate activity is learning how constructors, initializer lists, templates, inheritance, and header/CPP definitions fit together while building Subsystem and Superstructure around a reusable StateMachine.

## Project and environment

- Project: neblib, a team-maintained VEX V5 robotics library.
- Local workspace: `/home/cjda4/Documents/Code/VEX/neblib`.
- C++ standard: GNU C++11, VEXCode/VS Code VEX extension.
- SDK configured: `V5_20240802_15_00_00`.
- Last inspected commit: `14c3176` (`PRogress saved`). Working tree was clean immediately before this handoff file was created.
- The makefile compiles CPP files under `src`, `src/*`, and `src/*/*`. Therefore the example `template_subsystem.cpp` participates in the build.
- README development policy: use a GitHub issue and corresponding branch rather than developing on main; update documentation and README features when library features change.
- Robot hardware, sensors, mechanism states, ports, controls, and physical safety settings remain undecided. Do not invent them.
- At initial inspection, main.cpp was a competition-template prototype with an autonomous selector. Reinspect it before integration; it is not the source of a finalized robot configuration.
- neblib autonomous drive routines contain blocking loops. Do not call them in the periodic mechanism-owner loop.

## Important files

Paths are relative to the repository root so this document is portable:

| File | Purpose |
| --- | --- |
| `include/neblib/state_machine/state_machine.hpp` | Template definitions and state-machine behavior |
| `src/neblib/state_machine/state_machine.cpp` | Mostly includes; reusable template implementation lives in the header |
| `include/neblib/state_machine/subsystem.hpp` | Generic abstract subsystem base |
| `src/neblib/state_machine/template_subsystem.cpp` | Example concrete subsystem; currently compiled by the makefile |
| `include/superstructure/superstructure.hpp` | Concrete superstructure declaration and state enum |
| `src/superstructure/superstructure.cpp` | Concrete superstructure method definitions |
| `makefile`, `vex/mkenv.mk`, `vex/mkrules.mk` | VEX compilation configuration |
| `README.md`, `documentation/neblib/documentation.md` | Repository policy and documentation conventions |

Superstructure previously lived inside the neblib/state_machine directories. It has moved to the paths above; its namespace is still `neblib`.

## Agreed design

The user selected:

- Mechanism coordination for both autonomous and driver control, with driving kept separate.
- A reusable library feature, not only robot-specific logic.
- Enum states and entry/update/exit callbacks.
- Flat state machines: one active state per machine, no hierarchical states in the initial version.
- Caller-owned periodic updates, targeting roughly 10 ms; no background task per machine.
- Multiple independent mechanisms may have separate machines; tightly coordinated mechanisms may share one machine.

The registry originally used `std::unordered_map`. Actual enum-template instantiation failed with the installed VEX SDK because its standard library lacked the required default enum hash. A vector and custom hasher were discussed. The user chose `std::map`; that choice compiles with enum-class keys and needs no custom hasher. Do not recommend reverting it without a concrete reason.

`StateMachine<StateEnum>` currently has:

- Optional `std::function<void()>` entry, update, and exit callbacks, with default no-op lambdas.
- A nested `Registration<State>` template containing an enum and callbacks.
- Two constructors: registration plus clock reference; or map plus initial enum plus clock reference.
- A private map of registrations.
- A `uint32_t& robotTime`, read to capture state entry time. This is a caller-maintained clock, not automatically refreshed by the machine.
- Public `periodic`, `requestStateChange`, `start`, `stop`, two `reset` overloads, registration overloads, and some getters/setters.
- An `executingCallback` flag to reject lifecycle recursion.

Originally suggested callbacks received a machine reference and update accepted a timestamp. The actual implementation instead uses zero-argument callbacks/captures and a referenced clock. These are current design choices, not automatically errors. Document lifetime and ownership constraints.

## Original roadmap and step 2 contract

1. Establish the reusable interface: registration, start, transition requests, periodic update, reset, stop, inspection.
2. Implement predictable execution semantics.
3. Integrate with VEX competition modes and the robot's owner loop.
4. Configure actual mechanisms once hardware is known.
5. Validate and document the library and integration.

Step 2 was defined as follows:

- At most one transition per periodic cycle, before the active state's update callback.
- Transition order: old exit, change active state and entry timestamp, new entry, new update.
- Requests from callbacks are deferred to a later periodic cycle; no recursive transitions.
- First pending request wins. Reject additional requests until it is consumed.
- Consume old pending work before exit/entry callbacks so new requests can survive.
- Requests for the active state are successful no-ops when no earlier request occupies the slot. Reset explicitly supports same-state re-entry and a fresh timer.
- Unknown destinations and requests while stopped fail without mutation.
- Reset validates first, discards old pending work, exits if running, enters the destination, and resumes running.
- Stop exits once and discards pending work, including requests produced during its exit callback. Later periodic calls do nothing until restarted.
- Recursive lifecycle calls from callbacks are rejected; transition requests and inspection remain available.
- Callbacks are nonblocking. No sleeps, sensor-wait loops, or blocking drivetrain actions.
- All machine operations happen on one owning thread. The recursion flag is not a mutex.
- Use a consistent millisecond clock and unsigned subtraction for rollover-safe elapsed time. The referenced clock must outlive the machine.
- Robot callbacks own guards, timeouts, and fault/recovery logic. No generic nested-state or transition-rule framework is required.

Step 2 has not been fully signed off. Inspection and broader API hardening also remain incomplete. Copy/move prevention was in the original proposal but is not currently implemented; copying an object whose callbacks capture its address can be unsafe.

## Latest verified snapshot, October 21

Fresh VEX C++11 syntax checks were run while preparing this handoff. No production source was modified.

| Check | Result |
| --- | --- |
| Explicit instantiation of `StateMachine` with a small `enum class` | Passed |
| `src/superstructure/superstructure.cpp` | Failed: state-machine constructor missing arguments |
| `src/neblib/state_machine/template_subsystem.cpp` | Failed: base subsystem does not initialize its state machine; also a missing-override warning |

These were syntax checks, not a full linked project build, host behavior suite, or robot test.

### Current Superstructure findings

1. Include paths are now corrected. Methods are properly defined as `Superstructure::...` inside `namespace neblib` in the CPP.
2. The state machine is now in the constructor initializer list, fixing the earlier default-construction/assignment pattern. However, its constructor receives only a registration. It requires the clock reference too. The VEX compiler confirms this error at CPP line 7 in the inspected snapshot.
3. The initializer registers a dummy IDLE, then `registerStates()` attempts to register IDLE again. Duplicate rejection means the second set of callbacks cannot replace the first. Recommended design: initialize with an empty map, chosen initial enum, and clock; register every real state exactly once.
4. The constructor currently calls no-argument `reset()` to start. Recommended lifecycle is registration during construction and explicit startup after the complete object exists, especially when callbacks capture `this` or interact with other subsystems.
5. The header declares a second constructor accepting a StateMachine by value; no corresponding definition was found. It becomes a link error if used. Ask whether it is needed rather than inventing an implementation.

### Current Subsystem/example findings

1. `Subsystem<StateEnum>` has a state-machine member but initializes only its clock reference. StateMachine has no default constructor. This is the second compiler-confirmed blocker.
2. A reasonable interface is for the base constructor to receive an initial enum and clock and initialize an empty stopped machine; the concrete subsystem registers its states in its own constructor body.
3. `MySubsystem : Subsystem<SubsystemState>` currently uses private inheritance by default. Use public inheritance if it is intended to expose the base interface/polymorphism.
4. The example requests IDLE while the machine is stopped; such requests are rejected. Select the initial state at construction instead.
5. The example's `registerStates()` overrides a virtual method but lacks the `override` keyword; compiler warning only.
6. The base has no virtual destructor. Add one if ownership permits deletion through a base pointer; no need to introduce dynamic allocation just for this design.

### Current StateMachine findings from source inspection

Fixed since earlier reviews: map hashing compatibility; callback passing by const reference; flags initialized before initial registration; start marks running before entry; stop clears pending work; destination reset consumes pending work before exit; default timestamp initialized to zero; invalid startup rejected; lifecycle recursion guards present.

Remaining significant issue: no-argument `reset()` still duplicates logic incorrectly. It does not clear pending work, and when the current state is unregistered it skips callbacks but still sets running and returns success. The earlier unguarded entry dereference is now guarded; do not repeat that old exact complaint. Recommendation remains delegating to the validated `reset(currentState)` overload.

An invalid no-argument reset can also break the invariant assumed by destination reset: destination reset checks its destination but, if already marked running, assumes the previous state exists before dereferencing its lookup.

No-op defaults and callback existence checks are both present. Callback execution does not restore its guard if a callback throws; the VEX build uses `-fno-exceptions`, so treat exception behavior as a host-test consideration rather than the immediate robot blocker.

## C++ explanations already covered

- Header declares a class and methods; CPP defines concrete methods using `ClassName::methodName`. Do not redeclare the class in the CPP to implement it.
- Constructors have no return type. A header declaration ends with a semicolon; do not also provide a body there if defining the same concrete constructor in the CPP.
- Template definitions generally must be visible to callers, typically in the header or a file included by it. A separately compiled CPP does not support arbitrary new enum instantiations by itself.
- Construct members in the initializer list before the body. Assignment in the body happens too late for members without default constructors. Members initialize in header declaration order.
- StateMachine's reference member prevents its ordinary compiler-generated assignment operator from being usable.
- `class Intake : public neblib::Subsystem<IntakeState>` expresses public inheritance. A derived constructor initializes the base in its initializer list. `override` belongs on a matching virtual-method declaration in the derived class.
- Base constructors should not invoke virtual registration hooks expecting dispatch to the concrete derived implementation.
- `std::map` stores key/value pairs. `first` is the enum key; `second` is the registration value. `find()` returns an iterator; compare it against `end()` before dereferencing. A helper check must protect the actual lookup path, not just an earlier unrelated conditional.
- Map `operator[]` inserts a missing value. It is suitable for intentional registration insertion, not unchecked callback retrieval.
- A reference parameter uses `&` in its declaration. Do not append `&` to callback arguments at call sites.
- A callback guard rejects runtime recursive calls; it does not make public methods private. Set it around callbacks so legitimate internal periodic-to-reset calls remain possible.
- A virtual base destructor permits correct destruction through base pointers. Default destruction does not automatically stop robot motors.

## Recommended next interaction

Start with the Superstructure constructor's missing clock argument and the base Subsystem's missing state-machine initialization. Explain initializer-list syntax with minimal examples if requested. Preserve the user's responsibility for writing the implementation.

Then revisit duplicate IDLE registration, initial-state selection, explicit startup, and no-argument reset behavior. Recompile concrete source files and explicitly instantiate templates. Do not claim step 2 complete without lifecycle behavior tests.

Useful behavior tests once compiling:

- Registration/start/entry/update ordering; duplicate rejection; registration while running rejected.
- A pending A-to-B transition: A exit requests C, B enters and updates, next cycle transitions B-to-C.
- Entry and update requests are deferred; first pending request wins.
- Empty callbacks, invalid requests, current-state requests, explicit same-state reset.
- Reset clears older pending work while preserving allowed callback-generated new requests.
- Stop/restart never resumes stale pending work; repeated stop exits only once.
- Invalid initial state and reset fail without entering a running-invalid state.
- Callback lifecycle recursion rejected; two machines remain independent.
- Timer reset, irregular update intervals, unsigned rollover.

Later integration: single actuator owner; consistent sensor snapshot; fixed subsystem update order; competition-mode transitions clear stale commands and explicitly set hardware-safe outputs; mechanism updates remain responsive while autonomous driving runs elsewhere. Actual safe outputs require hardware configuration, not assumptions.

## Reproducing compiler checks on the original workstation

Installed compiler:

`/home/cjda4/.config/Code/User/globalStorage/vexrobotics.vexcode/tools/cpp/toolchain_linux64/clang/bin/clang`

SDK root for the makefile's `T` parameter:

`/home/cjda4/.config/Code/User/globalStorage/vexrobotics.vexcode/sdk/cpp/V5/V5_20240802_15_00_00`

Use `make -Bn` for the desired object target with `T` and `CXX` set to those paths to inspect the exact VEX flags. For the latest checks, the compile command's `-c -o ...` portion was replaced with `-fsyntax-only` and the source path. An enum-based explicit template instantiation was also supplied through compiler stdin. This tests methods that an include-only CPP may not instantiate.

A cloud or other machine will need its own VEX SDK/toolchain installation and paths. These absolute paths are workstation-specific. No hardware tests have been performed in this conversation.

## Portability and sharing

Give the next service this document plus access to the current source tree or the relevant files listed above. This document is not a replacement for the repository. The next service should not assume access to this chat, the local clock, the local filesystem, or the VEX toolchain.

Suggested opening instruction:

> Read CHAT_HANDOFF.md and the current project files. I am learning C++ and implementing the code myself. Explain and verify problems without editing source. Continue with the Subsystem and Superstructure constructor issues, then check the remaining state-machine lifecycle behavior. Treat the current files as authoritative over older findings.
