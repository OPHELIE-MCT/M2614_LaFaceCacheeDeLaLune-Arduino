---
description: Those instructions should load every time a task is assigned in the context of the Arduino Uno Q codebase for the M2614 project. They provide critical facts about the architecture, responsibilities, and development workflow of the project, as well as routing guidance for different types of tasks.
# applyTo: 'Describe when these instructions should be loaded by the agent based on task context' # when provided, instructions will automatically be added to the request context when the pattern matches an attached file
---

## Purpose

This is the always-on instruction file for the M2614 integration workspace.
Keep only critical facts here. Read the linked documents only when they are relevant to the assigned task.

## Critical Facts

- This repository is the integration project for M2614 "La face cachee de la lune". The local code is still mostly a skeleton. Validated behavior currently lives in prototype folders outside this directory.
- The robot is split across three execution domains:
  - Uno Q MCU: real-time control, global state machine, RC input, wheel encoders, wall switches, LiDAR, SPI link
  - Uno Q Linux SBC: Python services, RouterBridge RPC, dashboard, visualization, logging, heavy computation
  - Seeeduino Nano: ball sorter hardware, local sorter sensors, NeoPixel status display, emergency-stop reporting
- Keep the Uno Q as the SPI slave. This is a hard project constraint due to the current fragility of the Uno Q SPI stack.
- On Uno Q RouterBridge code, do not use `Serial.println(...)`. Use `Monitor.println(...)`, and call `Monitor.begin()` without a baud rate.
- Treat the Uno Q as a beta Arduino Zephyr target with a custom core installed at `C:\Users\david\AppData\Local\Arduino15\packages\arduino\hardware\zephyr\0.55.0\`.
- If for any reasons you need to edit the source code of the Uno Q core firmware, document the change and its rationale in the Markdown docs in this repository. Also notify the user that a new core firmware will need to be rebuilt and installed locally before the change can be tested. The user still need to compile this manually under WSL/Ubuntu, as the Uno Q development is Linux-first and may fail under Windows for reasons unrelated to the code change.
- Keep heavy, non-real-time work on the SBC side unless the task explicitly requires real-time execution on the MCU.
- Please note that communication between the Uno Q MCU and SBC done via RouterBridge is slow and has non-negligible latency. It is not suitable for high-frequency control loops or real-time sensor processing. Use it primarily for commands, configuration, and low-frequency telemetry as it can occasionally hang the MCU temporarily.

## Default Agent Behavior

- Start from the prototype that already owns the target behavior, then adapt that behavior into this integration project.
- Preserve existing protocol details when porting code: packet magic bytes, checksums, timing intervals, reset logic, and safety timeouts are part of the system contract.
- Prefer thin integration layers over large copy-paste merges of prototype sketches.
- If a change touches both MCU and SBC behavior, document the RPC or message contract in the Markdown docs in the same change.
- When compiling code for the Uno Q, do not use the VS Code tasks. Run the command by yourself so you can see the full output and debug any compilation issues that may arise. Run the same command as the one defined in the build task.
- Update [README.md](README.md) when the architecture, responsibilities, or development workflow materially change.
- When implementing a feature or patching code in header files, keep the formatting and style consistent with the existing code. Ensure every single function and/or methods is documented in a doxygen style docstring comment block, even if the function/method is private. This is critical for maintainability and knowledge transfer to future developers who may not have the context of the original implementation. Those docstrings must be kept updated if the edits impact the signature or behavior.
- The following fields are mandatory in docstrings:
  - `@brief` for every function/method, describing its purpose in one or two sentences
  - `@description` for every complex function/method that requires a more detailed explanation of its behavior, edge cases, or implementation details if quirks or ambiguities are expected because of hardware constraints or external dependencies. Try to avoid using `@description` if the behavior can be fully described in the `@brief` and code comments.
  - `@param` for every parameter, describing its purpose and expected values. If there is no parameter, don't include this field at all.
  - `@return` for every non-void function, describing the return value and its meaning. Exclude this field for void functions.
  - `@throws` for any function that can throw exceptions, describing the conditions under which exceptions are thrown and their types. Exclude this field if the function does not throw any exceptions.
  - `@author` for every function
  - `@date` for every function, indicating the date of the last update
  - omit completely fields that are not applicable instead of writing "none". For example, if a function does not throw any exceptions, does not return any value, and does not take any parameters, its docstring should only include the `@brief`, `@author`, and `@date` fields, etc.

## Task Routing

- RouterBridge or Uno Q Python/SBC task:
  read [../../docs/development-workflow.md](../../docs/development-workflow.md) and use `D:\ArduinoApps\color-data-gather\` as the primary reference.
- RC receiver, mecanum drive, or motor command task:
  use `C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code\Tests\RC_Reciever\` and `C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code\Tests\MechanumTest\`.
- LiDAR integration task:
  use `C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code\Tests\LiDAR\`.
- SPI integration between Uno Q and Seeeduino:
  use `C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code\Tests\SPI\` and preserve the Uno Q slave assumption.
- Ball sorting or sorter calibration task:
  use `C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code\ball-sorter\` and `C:\Users\david\Documents\dev\ball-analysis\`.

## Read On Demand

- [../../README.md](../../README.md) for the project overview and subsystem summary
- [../../docs/architecture.md](../../docs/architecture.md) for responsibilities, data flows, and non-negotiable constraints
- [../../docs/development-workflow.md](../../docs/development-workflow.md) for build/toolchain notes, task entry points, and integration order
