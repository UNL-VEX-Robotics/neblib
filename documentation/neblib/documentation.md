# neblib Documentation Guide

This page defines the conventions used by the neblib documentation. Following these conventions keeps each topic easy to read, accurate, and consistent as the library grows.

For an introduction to the documentation, begin with the [documentation welcome page](../README.md).

## Purpose

The documentation explains the public parts of neblib: the classes, functions, configuration values, and behaviors that users need when building a VEX V5 robot. It should help a reader understand both how to use a feature and the reasoning behind it.

The documentation does not replace the source code. The public header files in [`include/neblib`](../../include/neblib) are the source of truth for function signatures and class interfaces.

## File Organization

Documentation files are stored under `documentation/neblib`. In general, each page should correspond to a public header or a closely related group of features.

Use lowercase file names with underscores when more than one word is needed. For example:

```text
control_algorithms.md
position_tracking.md
standard_drive.md
```

Device-specific pages may be placed in a `devices` subdirectory that mirrors `include/neblib/devices`.

## Recommended Page Structure

Not every page needs every section, but most feature pages should use this order:

1. **Title** — the name of the class, module, or feature.
2. **Overview** — what the feature does and when it should be used.
3. **Requirements** — hardware, setup, units, or other prerequisites.
4. **API reference** — public classes, constructors, functions, and parameters.
5. **Example** — a small, practical usage example.
6. **Behavior and warnings** — side effects, limits, blocking behavior, and common mistakes.
7. **Related pages** — links to relevant documentation and source headers.

## Documenting an API

For each public class or function, document the details a user needs to call it correctly:

- its purpose;
- parameter names, meanings, and expected units;
- its return value;
- important default values;
- state that it changes;
- whether it blocks or starts a task;
- hardware or object-lifetime requirements; and
- errors, limitations, or unsafe inputs.

Use the exact names and capitalization found in the header file. Wrap code identifiers such as `neblib::clamp`, type names, parameter names, and file paths in backticks.

### Function Format

The following format can be adapted for individual functions:

````markdown
### `functionName`

```cpp
ReturnType functionName(ParameterType parameter);
```

Briefly explain what the function does.

**Parameters**

- `parameter` — Explain its meaning and units.

**Returns:** Explain the returned value.

**Notes:** Describe important behavior, limits, or side effects.
````

For a class, begin with its purpose and constructor, then give each public method its own subsection.

## Code Examples

Use fenced code blocks marked as C++:

```cpp
double angle = neblib::toRad(90.0);
```

Examples should be short enough to understand without unrelated robot setup. They should compile against the current public interface unless the example is explicitly labeled as pseudocode. Include the `neblib::` namespace when it helps show where an API comes from.

Never assume that a value's unit is obvious. State whether an example uses degrees, radians, inches, milliseconds, volts, or another unit.

## Writing Style

- Use clear, direct sentences and present tense.
- Explain acronyms and robotics terms the first time they appear.
- Describe observable behavior instead of internal implementation details unless those details help users make correct decisions.
- Use headings that describe their content rather than vague headings such as “Other.”
- Use relative Markdown links so links continue to work when the repository is viewed locally or on GitHub.
- Prefer one complete example over several nearly identical examples.

## Accuracy and Maintenance

Before completing a documentation change:

1. Compare names and signatures with the matching header file.
2. Check the implementation when behavior is not clear from the header.
3. Confirm that links resolve from the page's directory.
4. Check examples for correct types, units, and namespaces.
5. Update related pages if a renamed or changed API is mentioned elsewhere.

If the documentation and implementation disagree, treat the current code as authoritative and update the documentation. If the code itself appears incorrect, record or fix that issue separately rather than documenting behavior that does not exist.

## Page Template

Use this template when starting a new topic page:

````markdown
# Feature Name

## Overview

Explain what the feature does and when to use it.

## Requirements

List required setup, hardware, and units.

## API Reference

Document the public classes and functions.

## Example

```cpp
// Minimal usage example
```

## Behavior and Warnings

Describe limits, side effects, and common mistakes.

## Related Pages

- [Documentation title](relative/path.md)
- [Source header](../../include/neblib/header.hpp)
````

Return to the [documentation welcome page](../README.md) or read the [main project README](../../README.md) for a general overview of neblib.
