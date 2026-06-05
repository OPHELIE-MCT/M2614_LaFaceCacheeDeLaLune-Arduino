---
description: Those instructions should load every time a task is assigned in the context of the Arduino Uno Q codebase for the M2614 project. They provide critical facts about the architecture, responsibilities, and development workflow of the project, as well as routing guidance for different types of tasks.
# applyTo: 'Describe when these instructions should be loaded by the agent based on task context' # when provided, instructions will automatically be added to the request context when the pattern matches an attached file
---

## Purpose

This is the always-on instruction file for the M2614 integration workspace.
Keep only critical facts here. Read the linked documents only when they are relevant to the assigned task.

## Critical Facts

- This repository is the integration project for M2614 "La face cachee de la lune". The local code is still mostly a skeleton. Validated behavior currently lives in prototype folders outside this directory.
- Define `PROTO_ROOT = C:\Users\david\OneDrive - Education Vaud\M2614 La face cachée de la lune\10_Informatique\Code` and use it as the base path for prototype references in this file. If `PROTO_ROOT` is not accessible, stop and ask the user to provide the correct root path before continuing.
- The robot is split across three execution domains:
  - Uno Q MCU: real-time control, global state machine, RC input, wheel encoders, wall switches, LiDAR, SPI link
  - Uno Q Linux SBC: Python services, RouterBridge RPC, dashboard, visualization, logging, heavy computation
  - Seeeduino Nano: ball sorter hardware, local sorter sensors, NeoPixel status display, emergency-stop reporting
- Keep the Uno Q as the SPI slave. This is a hard project constraint due to the current fragility of the Uno Q SPI stack.
- On Uno Q RouterBridge code, do not use `Serial.println(...)`. Use `Monitor.println(...)`, and call `Monitor.begin()` without a baud rate.
- Treat the Uno Q as a beta Arduino Zephyr target with a custom core installed at `C:\Users\david\AppData\Local\Arduino15\packages\arduino\hardware\zephyr\0.55.0\`.
- If for any reasons you need to edit the source code of the Uno Q core firmware, verify that the core path `C:\Users\david\AppData\Local\Arduino15\packages\arduino\hardware\zephyr\0.55.0\` is referenced in the workspace before editing. If it cannot be confirmed, warn the user that core path availability is unverified and that the change is theoretical until the core is confirmed installed. Document the change and its rationale in the Markdown docs in this repository. Also notify the user that a new core firmware will need to be rebuilt and installed locally before the change can be tested. The user still need to compile this manually under WSL/Ubuntu, as the Uno Q development is Linux-first and may fail under Windows for reasons unrelated to the code change.
- Keep heavy, non-real-time work on the SBC side unless the task explicitly requires real-time execution on the MCU.
- Please note that communication between the Uno Q MCU and SBC done via RouterBridge is slow and has non-negligible latency. It is not suitable for high-frequency control loops or real-time sensor processing. Use it primarily for commands, configuration, and low-frequency telemetry as it can occasionally hang the MCU temporarily.
- If a requested feature would require crossing the RouterBridge at a frequency or latency that conflicts with the constraint above, do not implement it as described. Instead, flag the architectural conflict to the user, explain the latency constraint, and propose an alternative design that keeps high-frequency logic on the MCU side before writing any code.

## Default Agent Behavior

- Start from the prototype that already owns the target behavior, then adapt that behavior into this integration project.
- If a task spans multiple prototype directories, use all relevant prototypes as references and note any conflicts between them before proposing a solution. If the prototypes contradict each other, ask the user which takes precedence before proceeding.
- Preserve existing protocol details when porting code: packet magic bytes, checksums, timing intervals, reset logic, and safety timeouts are part of the system contract.
- Prefer thin integration layers over large copy-paste merges of prototype sketches.
- If a change touches both MCU and SBC behavior, document the RPC or message contract in the Markdown docs in the same change.
- When compiling code for the Uno Q, do not run the VS Code task runner UI. Execute the CLI command directly in the terminal so you can see full output and debug compilation issues, using the exact command configured in the Uno Q build task.
- Update [README.md](README.md) when any of the following change: the list of execution domains, the responsibilities assigned to a domain, the build/toolchain steps, or the communication protocols between subsystems. Do not update [README.md](README.md) for implementation-only changes that do not affect system boundaries or workflow.
- When implementing a feature or patching code in header files, keep the formatting and style consistent with the existing code. Ensure every single function and/or methods is documented in a doxygen style docstring comment block, even if the function/method is private. This is critical for maintainability and knowledge transfer to future developers who may not have the context of the original implementation. Those docstrings must be kept updated if the edits impact the signature or behavior.
- If a prototype path specified in Task Routing is not accessible, notify the user immediately with the full path that could not be found, explain that the task cannot be safely completed without the prototype reference, and ask the user to confirm the correct location before proceeding.
- The following fields are mandatory in docstrings and must be applied using this decision table:

  | Field | Include when | Omit when |
  | --- | --- | --- |
  | `@brief` | Always. Describe purpose in one or two sentences. | Never. |
  | `@details` | The function has hardware-specific timing constraints, depends on external protocol state, or has non-obvious side effects. | Behavior is fully captured by `@brief` and inline comments. When in doubt, omit `@details`. |
  | `@param` | The function has parameters; include one entry per parameter with expected values. | The function has no parameters. |
  | `@return` | The function is non-void; describe return meaning. | The function is void. |
  | `@throws` | The function can throw exceptions; describe conditions and types. | The function does not throw. |
  | `@author` | Always. | Never. |
  | `@date` | Always; set to the ISO 8601 date (`YYYY-MM-DD`) when the function was last modified by this agent session. If the current date is unavailable, write `@date UNKNOWN` and notify the user to fill it in. | Never. |

  Omit fields that are not applicable instead of writing "none".

## Task Routing

- RouterBridge or Uno Q Python/SBC task:
  read [../../docs/development-workflow.md](../../docs/development-workflow.md) and use `D:\ArduinoApps\color-data-gather\` as the primary reference.
- RC receiver, mecanum drive, or motor command task:
  use `${PROTO_ROOT}\Tests\RC_Reciever\` and `${PROTO_ROOT}\Tests\MechanumTest\`.
- LiDAR integration task:
  use `${PROTO_ROOT}\Tests\LiDAR\`.
- SPI integration between Uno Q and Seeeduino:
  use `${PROTO_ROOT}\Tests\SPI\` and preserve the Uno Q slave assumption.
- Ball sorting or sorter calibration task:
  use `${PROTO_ROOT}\ball-sorter\` and `C:\Users\david\Documents\dev\ball-analysis\`.

## Read On Demand

- [../../README.md](../../README.md) for the project overview and subsystem summary
- [../../docs/architecture.md](../../docs/architecture.md) for responsibilities, data flows, and non-negotiable constraints
- [../../docs/development-workflow.md](../../docs/development-workflow.md) for build/toolchain notes, task entry points, and integration order
