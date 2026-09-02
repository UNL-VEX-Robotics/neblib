# neblib

The "Nebraska Library," or `neblib`, is a library created and maintained by the UNLVEXU club at the University of Nebraska-Lincoln. It was created for use across VEX V5 Robotics competition programs. It uses C++11 and is intended for use with the VEX extension for Visual Studio Code.

## Features

* A feedback-controller interface and PID controller with configurable gains, behavior, settling conditions, output limits, and slew-rate limiting
* Position tracking with a pose interface, two-wheel odometry, a VEX Inertial Sensor, and a tracker-wheel abstraction for VEX Rotation Sensors
* Differential-drive control with tank and arcade inputs, turns, straight-line movements, and swing turns
* X-drive control with local, directional, and field-centric inputs, plus autonomous turns and movements to coordinates or poses
* A touchscreen autonomous selector with pages, selectable routines, and alliance-color selection
* A pneumatic-cylinder helper with set, toggle, and state-reading operations
* General-purpose task, angle-conversion, clamping, wrapping, random-number, sign, and string utilities

## Requirements for Use

### VEX Teams

As specified in [\<G2>](https://content.vexrobotics.com/docs/2026-2027/override/files/override-v1.1.pdf) of the GRSF Override Game Manual, students must follow the [GRSF student-centered policy](https://library.globalrobotics.org/hc/en-us/articles/51372675292820-Student-Centered-Policy#code-header-8). Teams using `neblib` must understand what the library does and why they are using it, and they must credit the original source. This is *not* an exhaustive list of team responsibilities. Read the [GRSF student-centered policy](https://library.globalrobotics.org/hc/en-us/articles/51372675292820-Student-Centered-Policy#code-header-8) for more information.

<small>Updated 08/26/2026. The GRSF Game Manual and Student-Centered Policy are owned by Innovation First, Inc.</small>

### RECF Teams

As specified in section 2.3 of both the [RECF Achieve: Pinnacle](https://kb.roboticseducation.org/hc/en-us/article_attachments/42292258600343) and [RECF Inspire: Pinnacle](https://kb.roboticseducation.org/hc/en-us/article_attachments/42292302172567) game manuals:

> Teams can use coding libraries as a starting point, but should modify the resulting code to make it their own. They should be able to explain how their code works and how it has been modified from the original ideas when asked. Outside references should be appropriately credited and described in teams’ engineering notebooks.

<small>Updated 08/26/2026. The RECF Achieve: Pinnacle and RECF Inspire: Pinnacle game manuals are owned by the Robotics Education & Competition Foundation.</small>

## Development

Development of `neblib` should *not* occur on the main branch. Create an issue through GitHub, and then create a branch for that issue. The branch name should be the automatically generated name assigned by GitHub. For example, the `21-update-readme` branch is for issue #21, titled "Update README."

When developing or changing functionality, update the Features section of this README if the change affects the library's features. Add or update the corresponding documentation in the `documentation` folder as well.

There should be absolutely zero AI-generated code used within `neblib`. Both RECF and VEX prevent AI-generated code from being used on a team's robot.

## Artificial Intelligence Use & Disclosure

Artificial Intelligence (AI) does not write any code for `neblib`.

ChatGPT has been used to improve the clarity and grammar of documentation files. These files include this README and files in the `documentation` folder.

## Notes and Warnings

The `main.cpp` file is used during prototyping.
Code not normally found within the VEX Competition Template can be deleted or written over with no consequence.