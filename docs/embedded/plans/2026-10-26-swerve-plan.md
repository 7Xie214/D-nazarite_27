# Swerve Kinematics Implementation Plan

> **For agentic workers:** REQUIRED: Use embedded-implementation skill to execute this plan task-by-task.

**Goal:** Implement portable four-module inverse kinematics with shortest steering.

**Architecture:** Chassis/Src/Swerve.c performs only C99 math on caller-owned
Swerve_Module_t objects. APP supplies calibrated angles and consumes mechanical
angle/speed targets; Device and BSP continue to own current commands and transport.

**Constraints:** Preserve existing module configuration fields. No HAL, RTOS,
motor types, allocations, hardware initialization or new task integration.

## Task 1: Establish executable behavior checks

Create `tests/test_swerve.c` and `tests/run_swerve_tests.ps1`. Test zero/negative RPM,
invalid radius, four-wheel translation/rotation, vector reconstruction for mixed
motion, 180-degree reversal, wrap crossing, multi-turn feedback, zero-speed angle
hold, speed saturation, bad counts/pointers/NaN/Inf/geometry and transactional
failure. Assert approximate floats with explicit tolerances. Exercise the current
conversion helper and report the missing kinematics API before implementation.

Run `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_swerve_tests.ps1`.
Expected initially: nonzero exit with a missing-kinematics or invalid-radius failure.

## Task 2: Implement the public contract

Modify `Chassis/Inc/Swerve.h`: document units, wheel index enum, status enum,
actual_angle_rad and function prototypes; keep existing configuration fields.
Modify `Chassis/Src/Swerve.c`: validate all inputs, calculate all four wheel vectors
into temporary arrays, optimize angles relative to feedback, hold stationary
headings, apply one common speed scale, then publish all results together.
Declare the existing RPM helper and validate its pointer/radius/result.

Run the native tests with `-std=c99 -Wall -Wextra -Werror -Wconversion -Wshadow
-Wstrict-prototypes -pedantic`; expected: zero failures. Compile Swerve.c using
ARMCC C99/Cortex-M7 settings and perform a complete Keil build.

## Task 3: Review and document usage

Review coordinate signs, physical units, stack/memory bounds and the absence of
hardware calls before reviewing C pointer/finite-value handling and API clarity.
Create `Chassis/README.md` with a four-module example, feedback/target contracts,
PID adaptation notes and native test command. Record fresh build/test results and
state the absence of board runtime verification. The directory is not a Git
repository, so no worktree or commit is created.

## Verification results

- Baseline before implementation: 2 checks, 2 failures (invalid-radius conversion
  and missing four-module API).
- Implemented library: 595 checks, 0 failures using the native GCC C99 runner
  with warnings treated as errors.
- ARMCC 5 Cortex-M7 C99 compile with warnings treated as errors: passed.
- Complete Keil build: 0 errors, 2 existing missing-final-newline warnings in
  algorithm/Src/PID.c and APP/Src/Communicate.c; AXF/HEX generated.
- Hardware/spec review and independent C-quality review: approved.
- No flash, board motion, feedback calibration or control-loop integration tested.
