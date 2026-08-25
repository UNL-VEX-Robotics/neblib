# Welcome to the neblib Documentation

This directory contains the documentation for **neblib**, a C++ library created by VURC team SKERS for VEX V5 robots. The library provides reusable tools for common robotics tasks so that teams can spend less time rewriting code between robots.

## About These Files

Each Markdown (`.md`) file explains a part of neblib. The documentation is intended to help readers:

- understand what each part of the library does;
- learn how to configure and use its classes and functions;
- find important requirements, parameters, and behavior;
- see examples that can be adapted for a VEX project; and
- understand the algorithms and control methods used by the library.

The documentation should be read alongside the header files in [`include/neblib`](../include/neblib). Header files define the public interface, while these guides provide the context and examples needed to use that interface effectively.

## Documentation Topics

- [`util.md`](neblib/util.md) documents general-purpose utilities.
- [`position_tracking.md`](neblib/position_tracking.md) explains poses, tracker-wheel setup, and odometry.
- [`control_algorithms.md`](neblib/control_algorithms.md) documents feedback controllers and PID configuration.
- [`standard_drive.md`](neblib/standard_drive.md) documents differential-drive controls and autonomous movements.
- [`documentation.md`](neblib/documentation.md) describes documentation-related conventions and guidance.

More pages will be added as the library is documented. Planned topics include other drive systems, autonomous selection, and device helpers.

## Who This Documentation Is For

These guides are written for VEX Robotics students and contributors who have a basic understanding of C++ and the VEX V5 API. Because teams using neblib should be able to explain the code and algorithms running on their robot, the documentation aims to explain both **how** to use a feature and **why** it works.

## Documentation Status

The documentation is a work in progress. Some pages may be incomplete or may change as neblib develops. When behavior described here differs from the current code, the code should be treated as the source of truth and the documentation should be updated.

## Contributing to the Documentation

When adding or updating a page:

1. Use a clear title and organize the page with descriptive headings.
2. Explain the purpose of the feature before listing implementation details.
3. Document public classes, functions, parameters, return values, and important side effects.
4. Include a short, practical example when possible.
5. Link to related documentation and source headers.
6. Keep terminology and formatting consistent with the other pages.

For a general overview of the project and its contribution requirements, see the [main README](../README.md).
